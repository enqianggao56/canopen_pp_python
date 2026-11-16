"""三台 Kinco 加 RS06 的四轴 PP 往返：共同 SYNC 启动，共同时间规划和全组停止。

RS06 不加入 Kinco 控制字映射，而在独立同步 RPDO 中接收绝对目标。
原三轴仍接收同一条共享控制字 RPDO；两个命令帧装载完毕后发送一次 SYNC。
"""

import logging
import sys
import time
from dataclasses import replace

from .group_controller import ThreeAxisController
from .group_model import plan_group, profile_time
from .group_transport import (CanopenGroupBus, SimulatedGroupBus, PDO_DISABLED,
                              PDO_SYNCHRONOUS, PDO_ASYNCHRONOUS)
from .objects import CW_HALT, CW_NEW_SETPOINT, Object
from .reciprocate import ReciprocationRunner
from .rs06 import RS06Peer, SimulatedRS06, RS06_COUNTS_PER_REV
from .transport import CanopenTransport, TransportError
from .pdo_feedback import CyclicFeedback

LOG = logging.getLogger(__name__)
SYNC_COB_ID = 0x80
SYNC_COB_OBJECT = 0x1005
SYNC_PERIOD_OBJECT = 0x1006
TARGET_POSITION_MAPPING = (int(Object.TARGET_POSITION) << 16) | 32
# 满速600RPM时10inc约15µs；给绝对位置的再次采样留0.1ms余量，
# 避免静止1~2inc抖动使严格最短时间校验误报。该余量计入模型，不代表实机精度。
ABSOLUTE_POSITION_TIME_MARGIN_S = 0.0001


def validate_topology(settings, rs06):
    """只校验四轴唯一节点、总线及固定起点，不连接设备；错误抛 ValueError。"""
    if rs06.node_id in {axis.settings.node_id for axis in settings.axes}:
        raise ValueError('RS06 节点 ID 与 Kinco 重复')
    if rs06.target_cob_id == settings.shared_cob_id:
        raise ValueError('RS06 目标 RPDO 与 Kinco 控制字不能使用相同 COB-ID')
    for axis in settings.axes:
        config = axis.settings
        if (config.interface, config.channel, config.bitrate) != (rs06.interface, rs06.channel, rs06.bitrate):
            raise ValueError('四轴必须使用相同 CAN 接口、通道和波特率')
        if axis.origin_position_inc is None:
            raise ValueError('四轴往返必须逐轴配置固定 origin_position_inc')


class FourAxisBus(CanopenGroupBus):
    """借用同一个 CANopen Network；保存四轴映射及 SYNC 周期，停稳后恢复。"""

    def __init__(self, settings, rs06, sleep=time.sleep):
        """保存两种设备配置；不连接设备，不修改 PDO。"""
        validate_topology(settings, rs06)
        super().__init__(settings)
        self.rs06_settings, self.sleep = rs06, sleep
        # 先用异步通道完成SDO状态/定位方式锁存，全部装载后再统一切为同步。
        self.transmission_type = PDO_ASYNCHRONOUS
        self.rs06_io = None
        self.peer = None
        self._rs_changed = None
        self._sync_changed = []
        self._other_rpdo_changed = []
        self._cyclic_sync_active = False
        self._motion_triggered = False
        self.sync_period_s = None
        self.feedback = None

    def open(self):
        """连接共享网络，再添加节点4；任何失败释放连接，不发送状态/运动命令。"""
        try:
            super().open()
            self.rs06_io = CanopenTransport(self.rs06_settings, network=self.network)
            self.rs06_io.open()
        except Exception:
            self.close()
            raise

    def verify_identities(self):
        """只读核对四台设备与各自 EDS 身份；RS06 失败不允许三轴先运行。"""
        return super().verify_identities() + (self.rs06_io.verify_identity(),)

    def check_sync_configuration(self):
        """只读检查四轴 SYNC COB-ID 和 PDO 冲突，返回 RS06 原始映射快照。

        要求 Kinco 0x1005=0x80；RS06 允许初始0，运行时临时设为0x80并回读。
        所有轴必须作为消费者、不自行产生SYNC；现场还须确认没有其他
        SYNC 主站。不能靠 SDO 检查证明总线独占性。
        """
        snapshots = self._snapshot(self.rs06_io)
        for io in self.axes + [self.rs06_io]:
            allowed = (0, SYNC_COB_ID) if io is self.rs06_io else (SYNC_COB_ID,)
            if io.read(SYNC_COB_OBJECT) not in allowed:
                raise TransportError(f'节点 {io.settings.node_id} 的 0x1005 必须为 0x80 消费者')
            io.read(SYNC_PERIOD_OBJECT)
            for index in io.node.object_dictionary:
                if not (0x1400 <= index <= 0x1407 or 0x1800 <= index <= 0x1807):
                    continue
                if index == self.comm_index:
                    continue
                cob_id = io.read(index, 1)
                if not cob_id & PDO_DISABLED and cob_id & 0x1FFFFFFF in (
                        self.settings.shared_cob_id, self.rs06_settings.target_cob_id):
                    raise TransportError(f'节点 {io.settings.node_id} 的其他 PDO 占用四轴预留 COB-ID')
        return snapshots

    def configure(self):
        """预先完成全组冲突读取，再配置同步 RPDO；失败保留快照供全组清理恢复。"""
        if not self.rs06_settings.synchronous_pdo_verified:
            raise TransportError('须确认 RS06 同步 RPDO、SYNC 独占及槽位后设置 synchronous_pdo_verified')
        if (self._rs_changed is not None or self._sync_changed or self._other_rpdo_changed
                or (self.feedback is not None and (self.feedback.changed or self.feedback.subscriptions))):
            raise TransportError('四轴仍有未恢复配置，拒绝重复映射')
        snapshot = self.check_sync_configuration()
        # 保留的旧RPDO1可能也映射607A/6060/6040。共同SYNC前先暂时禁用
        # 其他写入这些对象的RPDO，避免多个通道向同一控制对象送入旧过程数据。
        for io in self.axes + [self.rs06_io]:
            for index in io.node.object_dictionary:
                if not 0x1400 <= index <= 0x1407 or index == self.comm_index:
                    continue
                cob_id = io.read(index, 1)
                if cob_id & PDO_DISABLED:
                    continue
                mapping_index = index + 0x200
                count = io.read(mapping_index, 0)
                mappings = [io.read(mapping_index, sub) for sub in range(1, count+1)]
                if any(mapping >> 16 in (Object.CONTROLWORD, Object.MODE, Object.TARGET_POSITION)
                       for mapping in mappings):
                    transmission = io.read(index, 2)
                    self._other_rpdo_changed.append((io, index, cob_id, transmission))
                    io.node.nmt.state = 'PRE-OPERATIONAL'
                    io.write(index, cob_id | PDO_DISABLED, 1)
                    io.write(index, PDO_ASYNCHRONOUS, 2)
                    if io.read(index, 1) != cob_id | PDO_DISABLED:
                        raise TransportError(f'节点{io.settings.node_id} 其他控制RPDO禁用回读失败')
                    if io.read(index, 2) != PDO_ASYNCHRONOUS:
                        raise TransportError(f'节点{io.settings.node_id} 其他控制RPDO关闭同步门控失败')
        super().configure()
        self._rs_changed = snapshot
        self.rs06_io.node.nmt.state = 'PRE-OPERATIONAL'
        self.sleep(self.rs06_settings.poll_interval_s)
        expected = (self.rs06_settings.target_cob_id, PDO_SYNCHRONOUS, (TARGET_POSITION_MAPPING,))
        self._write_mapping(self.rs06_io, *expected)
        if self._snapshot(self.rs06_io) != expected:
            raise TransportError('RS06 同步目标 RPDO 回读不匹配')
        for io in self.axes + [self.rs06_io]:
            # 保留实机现有SYNC周期。Kinco当前固件不接受1006=0，不能
            # 通过忽略回读差异来继续；PP共同触发只改变RPDO类型和必要的SYNC地址。
            original = io.read(SYNC_PERIOD_OBJECT)
            original_cob_id = io.read(SYNC_COB_OBJECT)
            self._sync_changed.append((io, original, original_cob_id))
            if original_cob_id != SYNC_COB_ID:
                io.write(SYNC_COB_OBJECT, SYNC_COB_ID)
                if io.read(SYNC_COB_OBJECT) != SYNC_COB_ID:
                    raise TransportError(f'节点 {io.settings.node_id} SYNC地址回读不匹配')
        feedback_axes = self.axes + [self.rs06_io]
        self.feedback = CyclicFeedback(self.network, feedback_axes, feedback_axes,
                                       self.settings, lambda: self._cyclic_sync_active,
                                       remap_state_axes=(self.rs06_io,))
        self.feedback.configure()

    def start_all(self):
        """启动Kinco；RS06保持预操作态完成SDO准备，到共同触发前才切入运行态。"""
        super().start_all()

    def rs06_preoperational(self):
        """进入预操作态并关闭RS06同步门控，使SDO状态控制立即生效。

        实机即使在NMT预操作态，传输类型1仍会让SDO控制字等待SYNC。
        因此先禁用目标RPDO，再改回异步254；不恢复原映射、不发SYNC或位置目标。
        仅在共同启动前重启同步映射，停止时保持通道禁用至确认全组静止。
        """
        self.rs06_io.node.nmt.state = 'PRE-OPERATIONAL'
        self.sleep(self.rs06_settings.poll_interval_s)
        cob_id = self.rs06_io.read(self.comm_index, 1)
        self.rs06_io.write(self.comm_index, cob_id | PDO_DISABLED, 1)
        self.rs06_io.write(self.comm_index, PDO_ASYNCHRONOUS, 2)
        self.sleep(self.rs06_settings.poll_interval_s)
        if self.rs06_io.read(self.comm_index, 2) != PDO_ASYNCHRONOUS:
            raise TransportError('RS06停止/准备时关闭同步门控回读失败')

    def _start_rs06(self):
        """RS06已在预操作态使能并保持当前位置后，切入运行态接收同步目标。"""
        expected = (self.rs06_settings.target_cob_id, PDO_SYNCHRONOUS, (TARGET_POSITION_MAPPING,))
        self._write_mapping(self.rs06_io, *expected)
        if self._snapshot(self.rs06_io) != expected:
            raise TransportError('RS06共同触发前同步映射回读失败')
        self.rs06_io.start()
        self.sleep(self.rs06_settings.poll_interval_s)
        # 当前Revision0x500实机进入OP后需先收到一条初始化SYNC，
        # 才接收后续同步RPDO。此时Kinco仍为Halt，尚未发送任何新目标帧。
        self._send(SYNC_COB_ID, b'')
        self.sleep(self.rs06_settings.poll_interval_s)

    def _arm_kinco_sync(self, controlword):
        """三轴定位方式及目标已在异步准备阶段锁存后，切为同一同步控制字通道。"""
        # 先通过同一RPDO让实际过程控制字锁存定位方式和Halt；不置bit4。
        # 当前实机SDO与PDO控制字缓存不同，不能仅用SDO回读推断PDO已采用相对方式。
        CanopenGroupBus.broadcast(self, (controlword & ~CW_NEW_SETPOINT) | CW_HALT)
        self.sleep(self.rs06_settings.poll_interval_s)
        expected = (self.settings.shared_cob_id, PDO_SYNCHRONOUS,
                    ((int(Object.CONTROLWORD) << 16) | 16,))
        for io in self.axes:
            self._write_mapping(io, *expected)
            if self._snapshot(io) != expected:
                raise TransportError(f'节点{io.settings.node_id} 触发前同步映射回读失败')

    def _send(self, cob_id, data):
        """发送有界等待的标准帧；不重试目标或 SYNC，以免重复不确定命令。"""
        import can
        self.network.check()
        timeout = min(self.rs06_settings.sdo_timeout_s,
                      *(axis.settings.sdo_timeout_s for axis in self.settings.axes))
        message = can.Message(arbitration_id=cob_id, is_extended_id=False, data=data)
        with self.network.send_lock:
            self.network.bus.send(message, timeout=timeout)

    def _stop_cyclic_sync(self):
        """停止周期SYNC，防止停止/重新装载阶段有后台SYNC抢先释放目标。"""
        if self._cyclic_sync_active:
            self.network.sync.stop()
            self._cyclic_sync_active = False

    def _start_cyclic_sync(self):
        """新目标已共同触发后立即按设备原周期维持同步通信，覆盖PP应答阶段。"""
        if not self._cyclic_sync_active:
            periods = [period / 1000000 for _, period, _ in self._sync_changed if period > 0]
            self.sync_period_s = min(periods) if periods else self.rs06_settings.poll_interval_s
            self.network.sync.start(self.sync_period_s)
            self._cyclic_sync_active = True

    def broadcast(self, controlword):
        """装载两种命令后统一 SYNC；停止时先停止 RS06，避免释放残留待处理目标。

        CAN 发帧后等待配置的发送余量再发高优先级 SYNC，避免 SYNC 抢先。
        余量依赖总线负载，必须实机核对；不是硬实时发送完成保证。
        """
        if self.peer is None:
            raise TransportError('RS06 伴随控制器尚未绑定')
        stop_error = None
        if controlword & CW_HALT:
            self._stop_cyclic_sync()
            self._motion_triggered = False
            try:
                self.peer.request_halt()
            except Exception as exc:
                stop_error = exc
        elif controlword & CW_NEW_SETPOINT:
            self._arm_kinco_sync(controlword)
            self._start_rs06()
            self._send(self.rs06_settings.target_cob_id, self.peer.target.to_bytes(4, 'little', signed=True))
        super().broadcast(controlword)
        if stop_error is not None:
            # 仍发送 Kinco Halt RPDO，但不以 SYNC 释放未确认停止的 RS06 残留目标。
            # 控制器随后继续向每台 Kinco 和 RS06 分别发送 SDO 停止请求。
            raise TransportError(f'RS06 停止请求失败，未发送 SYNC：{stop_error}') from stop_error
        self.sleep(self.rs06_settings.sync_settle_s)
        if controlword & CW_NEW_SETPOINT:
            self.feedback.reset()
        self._send(SYNC_COB_ID, b'')
        if controlword & CW_NEW_SETPOINT:
            self._motion_triggered = True
            # 首轮反馈/应答读取也需要周期SYNC，不能等应答复位后才启动。
            # 只重复SYNC，不重发运动PDO，因此没有新的bit4上升沿。
            self._start_cyclic_sync()
        elif not controlword & CW_HALT and self._motion_triggered:
            # 按Kinco手册§9.3.3.3维持周期SYNC；不重发bit4上升沿或运动PDO。
            self._start_cyclic_sync()

    def restore(self):
        """仅由全组确认停稳后的调用方进入；逐项恢复，失败不跳过其余配置。"""
        self._stop_cyclic_sync()
        errors = []
        try:
            super().restore()
        except Exception as exc:
            errors.append(str(exc))
        if self._rs_changed is not None:
            try:
                self.rs06_io.node.nmt.state = 'PRE-OPERATIONAL'
                self._write_mapping(self.rs06_io, *self._rs_changed)
                if self._snapshot(self.rs06_io) != self._rs_changed:
                    raise TransportError('RS06 原映射回读不匹配')
                self.rs06_io.start()
                self._rs_changed = None
            except Exception as exc:
                errors.append(f'RS06 映射恢复：{exc}')
        failed = []
        for io, original, original_cob_id in self._sync_changed:
            try:
                if io.read(SYNC_PERIOD_OBJECT) != original:
                    io.write(SYNC_PERIOD_OBJECT, original)
                if io.read(SYNC_PERIOD_OBJECT) != original:
                    raise TransportError('回读不匹配')
                if io.read(SYNC_COB_OBJECT) != original_cob_id:
                    io.write(SYNC_COB_OBJECT, original_cob_id)
                if io.read(SYNC_COB_OBJECT) != original_cob_id:
                    raise TransportError('SYNC地址恢复回读不匹配')
            except Exception as exc:
                failed.append((io, original, original_cob_id))
                errors.append(f'节点{io.settings.node_id} SYNC周期恢复：{exc}')
        self._sync_changed = failed
        failed_other = []
        for io, index, cob_id, transmission in self._other_rpdo_changed:
            try:
                io.write(index, transmission, 2)
                io.write(index, cob_id, 1)
                if io.read(index, 1) != cob_id or io.read(index, 2) != transmission:
                    raise TransportError('原COB-ID回读不匹配')
            except Exception as exc:
                failed_other.append((io, index, cob_id, transmission))
                errors.append(f'节点{io.settings.node_id} 其他RPDO恢复：{exc}')
        self._other_rpdo_changed = failed_other
        if self.feedback is not None:
            try:
                self.feedback.restore()
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise TransportError('四轴配置恢复失败：' + '; '.join(errors))

    def close(self):
        """释放连接前停止后台SYNC；驱动器停止仍由控制器负责。"""
        self._stop_cyclic_sync()
        if self.feedback is not None:
            self.feedback.detach()
        super().close()


class SimulatedFourAxisBus(SimulatedGroupBus):
    """四轴协议模拟；保留三个 Kinco 的共同帧，RS06 仅接收自己的绝对目标。"""

    def __init__(self, initial_positions=(0, 0, 0), rs06_position=0):
        """初始化独立 RS06 模拟器；events 记录同一逻辑 SYNC 的四轴释放。"""
        super().__init__(initial_positions)
        self.rs06_io = SimulatedRS06(rs06_position)
        self.peer = None
        self.events = []

    def broadcast(self, controlword):
        """模拟同步装载和释放；不模拟真实总线帧发送延迟或机械运动时间。"""
        if controlword & CW_HALT:
            self.peer.request_halt()
        super().broadcast(controlword)
        if controlword & CW_NEW_SETPOINT and not controlword & CW_HALT:
            self.rs06_io.write(Object.TARGET_POSITION, self.peer.target)
            self.events.append(('sync_start', tuple(io.target for io in self.axes), self.peer.target))


class FourAxisController(ThreeAxisController):
    """补充四轴轮询观察时差；三轴仍由基类统一装载、握手与联停。"""

    def run(self):
        """正常完成时返回原三轴结果，并记录包含 RS06 到位确认的主机观察时差。"""
        starts = tuple(io.read(Object.ACTUAL_POSITION) for io in self.bus.axes)
        result = super().run()
        peer = self.peers[0]
        arrivals = [arrival for arrival, axis, start in zip(result.observed_arrival_times_s, result.plan.axes, starts)
                    if abs(axis.end_inc-start) > axis.axis.settings.position_tolerance_inc]
        if peer.has_motion:
            arrivals.append(peer.arrived_at)
        self.four_axis_observed_spread_s = max(arrivals) - min(arrivals) if arrivals else 0.0
        return result


class FourAxisReciprocationRunner(ReciprocationRunner):
    """四轴每程共同开始、按共同时间规划，全部到位和停稳后换向。"""

    def __init__(self, bus, settings, parameters, rs06,
                 clock=time.monotonic, sleep=time.sleep, sample_sink=None):
        """复用原三轴计数/端点逻辑；第四轴保留独立恢复段与完整单程计数。"""
        validate_topology(settings, rs06)
        self.peer = RS06Peer(bus.rs06_io, rs06, clock, sleep,
                            enter_preoperational=getattr(bus, 'rs06_preoperational', None))
        bus.peer = self.peer
        self.coordination = []
        self.last_controller = None
        super().__init__(bus, settings, parameters, clock, sleep, sample_sink,
                         controller_factory=self._controller)

    def _controller(self, bus, settings, clock, sleep, poll_hook):
        """每程使用三轴共享触发控制器，并将第四轴纳入监视、到位等待和全组停止。"""
        if self.last_controller is not None:
            self._record_coordination()
        self.last_controller = FourAxisController(bus, settings, clock, sleep, poll_hook, peers=(self.peer,))
        self.last_controller.model_spread_s = self.model_spread_s
        return self.last_controller

    def _record_coordination(self):
        """分别保留整数模型时差和主机轮询时差；未完成程不伪造到位时间。"""
        controller = self.last_controller
        if controller is not None and hasattr(controller, 'four_axis_observed_spread_s'):
            self.coordination.append(dict(model_arrival_spread_s=controller.model_spread_s,
                observed_arrival_spread_s=controller.four_axis_observed_spread_s))
            self.coordination = self.coordination[-100:]
            LOG.info('四轴量化模型到达时差 %.6fs，轮询观察到达时差 %.6fs（含通信和确认窗口）',
                     controller.model_spread_s, controller.four_axis_observed_spread_s)
        self.last_controller = None

    def _feedback_axes(self):
        """按节点配置顺序输出四轴反馈，RS06 速度使用自己的 0.1 rpm 单位。"""
        return super()._feedback_axes() + [(self.peer.io, self.peer.settings)]

    def _leg_settings(self, endpoints, starts=None):
        """计算共同时间并量化两种设备轮廓；原三轴继续保持同一时间规划。"""
        if starts is None:
            starts = tuple(io.read(Object.ACTUAL_POSITION) for io in self.bus.axes)
        settings = super()._leg_settings(endpoints, starts)
        # 四轴实机以原已确认编码器坐标直接指定固定端点。同步RPDO下相对
        # 指令接管暂停旧轨迹的行为未通过实测，不能继续依赖相对方式推断终点。
        # 不更改零位、原端点或限位；真实配置须明确确认绝对坐标有效。
        settings = replace(settings, axes=tuple(replace(axis,
            motion=replace(axis.motion, target_inc=endpoint, relative=False))
            for axis, endpoint in zip(settings.axes, endpoints)))
        for axis, start in zip(settings.axes, starts):
            axis.motion.validate(axis.settings, start)
        first = self.settings.axes[0]
        far = first.origin_position_inc + round(self.parameters.turns * first.settings.encoder_counts_per_rev)
        forward = endpoints[0] == far
        target = self.peer.settings.far_position_inc if forward else self.peer.settings.origin_position_inc
        # 同一次规划复用同一位置快照，避免静止1~2inc抖动使两次读取的
        # 最短时间互相矛盾，误报共同时间过短。
        rs_start = self.peer.check()['start_position_inc']
        rs_minimum = self.peer.plan(target, start_position_inc=rs_start)
        kinco_minimum = plan_group(settings, starts).duration_s
        common = max(rs_minimum, kinco_minimum)
        if common:
            common += ABSOLUTE_POSITION_TIME_MARGIN_S
        settings = replace(settings, duration_s=common if common else None)
        plan = plan_group(settings, starts)
        rs_duration = self.peer.plan(target, common, start_position_inc=rs_start)
        moving = [axis.estimated_duration_s for axis, start in zip(plan.axes, starts) if axis.end_inc != start]
        if rs_duration:
            moving.append(rs_duration)
        self.model_spread_s = max(moving) - min(moving) if moving else 0
        LOG.info('四轴共同规划 %.6fs，RS06 %.3f RPM，量化模型到达时差 %.6fs',
                 common, self.peer.velocity_raw / 10, self.model_spread_s)
        return settings

    def check(self):
        """只读检查四轴端点、首程恢复及完整正反两程，均在第一次使能前完成。"""
        checked = super().check()
        rs_checked = self.peer.check()
        # 从中途恢复的短距离可行，不代表完整100圈也能在配置超时内完成。
        distance = (self.peer.settings.far_position_inc - self.peer.settings.origin_position_inc) / RS06_COUNTS_PER_REV
        velocity = self.peer.settings.velocity_raw(self.peer.settings.velocity_rpm) / 600
        acceleration = self.peer.settings.acceleration_raw(self.peer.settings.velocity_rpm / 60 /
                                                          self.peer.settings.acceleration_time_s) / 600
        rs_full, _ = profile_time(distance, velocity, acceleration, acceleration)
        if rs_full >= self.peer.settings.motion_timeout_s:
            raise ValueError('RS06 完整往返单程超过运动超时')
        for endpoints, starts in ((checked['far_positions_inc'], checked['origin_positions_inc']),
                                  (checked['origin_positions_inc'], checked['far_positions_inc'])):
            full_leg = super()._leg_settings(endpoints, starts)
            common = max(rs_full, plan_group(full_leg, starts).duration_s)
            plan_group(replace(full_leg, duration_s=common), starts)
            if common >= self.peer.settings.motion_timeout_s:
                raise ValueError('四轴完整单程共同时间超过 RS06 运动超时')
        checked['rs06'] = rs_checked
        self._leg_settings(checked['first_target_positions_inc'], checked['start_positions_inc'])
        self.peer.preflight()
        return checked

    def run(self):
        """运行四轴并追加第四轴计数；异常时控制器先请求全部停止再逐轴确认。"""
        try:
            return super().run()
        finally:
            self._record_coordination()
            self.report['rs06_completed_legs'] = self.peer.completed_legs
            self.report['rs06_completed_full_legs'] = self.peer.completed_full_legs
            self.report['rs06_completed_round_trips'] = self.peer.completed_full_legs // 2
            self.report['four_axis_model_arrival_spread_s'] = getattr(self, 'model_spread_s', None)
            self.report['four_axis_recent_coordination'] = self.coordination
            self.report['hardware_validated'] = False


def main(argv=None):
    """独立四轴入口；--rs06-config 可覆盖第四轴配置，其余参数沿用三轴往返入口。"""
    from .reciprocate import main as reciprocate_main
    return reciprocate_main(['--config', 'config/four_axis.kinco.example.json',
                            '--rs06-config', 'config/rs06.example.json']
                            + list(sys.argv[1:] if argv is None else argv))


if __name__ == '__main__':
    raise SystemExit(main())
