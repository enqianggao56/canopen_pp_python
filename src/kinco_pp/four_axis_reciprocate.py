"""三台Kinco与RobStride伴随轴的PP往返；保留原四轴入口，七轴复用本模块。

RobStride伴随轴不加入Kinco控制字映射，而在独立同步 RPDO 中接收绝对目标。
原三轴仍接收同一条共享控制字RPDO；全部伴随轴目标装载完毕后发送一次SYNC。
"""

import logging
import sys
import time
from dataclasses import replace

from .group_controller import ThreeAxisController
from .group_model import plan_group
from .group_transport import (CanopenGroupBus, SimulatedGroupBus, PDO_DISABLED,
                              PDO_SYNCHRONOUS, PDO_ASYNCHRONOUS)
from .objects import CW_HALT, CW_NEW_SETPOINT, Object
from .reciprocate import ReciprocationRunner
from .rs06 import RS06Peer, SimulatedRS06
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


def validate_topology(settings, rs06, extra_settings=()):
    """只校验全部轴唯一节点、独立目标COB-ID、共用总线及固定起点，不访问设备。"""
    companions = (rs06,) + tuple(extra_settings)
    nodes = [axis.settings.node_id for axis in settings.axes] + [axis.node_id for axis in companions]
    if len(nodes) != len(set(nodes)):
        raise ValueError('RobStride 节点 ID 与其他轴重复')
    cob_ids = [settings.shared_cob_id] + [axis.target_cob_id for axis in companions]
    if len(cob_ids) != len(set(cob_ids)):
        raise ValueError('各轴目标 RPDO 与 Kinco 控制字不能使用相同 COB-ID')
    bus = (rs06.interface, rs06.channel, rs06.bitrate)
    for config in [axis.settings for axis in settings.axes] + list(companions):
        if (config.interface, config.channel, config.bitrate) != bus:
            raise ValueError('全部轴必须使用相同 CAN 接口、通道和波特率')
    if any(axis.origin_position_inc is None for axis in settings.axes):
        raise ValueError('混合多轴往返必须逐轴配置固定 origin_position_inc')


class FourAxisBus(CanopenGroupBus):
    """共享CANopen网络管理三台Kinco及1至4台RobStride，停稳后恢复配置。"""

    def __init__(self, settings, rs06, sleep=time.sleep, extra_settings=()):
        """保存Kinco与逐轴RobStride配置；不连接设备，不修改PDO。"""
        validate_topology(settings, rs06, extra_settings)
        super().__init__(settings)
        self.rs06_settings, self.sleep = rs06, sleep
        # 先用异步通道完成SDO状态/定位方式锁存，全部装载后再统一切为同步。
        self.transmission_type = PDO_ASYNCHRONOUS
        self.extra_settings = tuple(extra_settings)
        self.extra_ios, self.extra_peers = [], []
        self.rs06_io = None
        self.peer = None
        self._rs_changed = []
        self._sync_changed = []
        self._other_rpdo_changed = []
        self._cyclic_sync_active = False
        self._motion_triggered = False
        self.sync_period_s = None
        self.feedback = None

    @property
    def robstride_ios(self):
        """按配置顺序返回RS06及新增轴的通信对象。"""
        return [self.rs06_io] + self.extra_ios

    @property
    def robstride_peers(self):
        """返回全部伴随控制器，用于共同装载和逐轴停止。"""
        return [self.peer] + self.extra_peers

    def open(self):
        """添加所有节点到共享网络；任何失败释放连接，不发送状态/运动命令。"""
        try:
            super().open()
            self.rs06_io = CanopenTransport(self.rs06_settings, network=self.network)
            self.rs06_io.open()
            for config in self.extra_settings:
                io = CanopenTransport(config, network=self.network)
                self.extra_ios.append(io)
                io.open()
        except Exception:
            self.close()
            raise

    def verify_identities(self):
        """只读核对全部设备与各自EDS身份；任一轴失败不允许其余轴先运行。"""
        return super().verify_identities() + tuple(io.verify_identity() for io in self.robstride_ios)

    def check_sync_configuration(self):
        """只读检查全部轴SYNC/目标PDO冲突，返回各RobStride IO及原映射快照。

        要求Kinco 0x1005=0x80；RobStride允许初始0，运行时临时设为0x80并回读。
        所有轴必须作为消费者、不自行产生SYNC；现场还须确认没有其他
        SYNC 主站。不能靠 SDO 检查证明总线独占性。
        """
        snapshots = [(io, self._snapshot(io)) for io in self.robstride_ios]
        for io in self.axes + self.robstride_ios:
            allowed = (0, SYNC_COB_ID) if io in self.robstride_ios else (SYNC_COB_ID,)
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
                        self.settings.shared_cob_id, *(axis.settings.target_cob_id for axis in self.robstride_ios)):
                    raise TransportError(f'节点 {io.settings.node_id} 的其他 PDO 占用全组预留 COB-ID')
        return snapshots

    def configure(self):
        """预先完成全组冲突读取，再配置同步 RPDO；失败保留快照供全组清理恢复。"""
        if not all(config.synchronous_pdo_verified for config in (self.rs06_settings,) + self.extra_settings):
            raise TransportError('须逐轴确认 RobStride 同步 RPDO/TPDO、SYNC 独占及槽位后设置 synchronous_pdo_verified')
        if (self._rs_changed or self._sync_changed or self._other_rpdo_changed
                or (self.feedback is not None and (self.feedback.changed or self.feedback.subscriptions))):
            raise TransportError('全组仍有未恢复配置，拒绝重复映射')
        snapshots = self.check_sync_configuration()
        # 保留的旧RPDO1可能也映射607A/6060/6040。共同SYNC前先暂时禁用
        # 其他写入这些对象的RPDO，避免多个通道向同一控制对象送入旧过程数据。
        for io in self.axes + self.robstride_ios:
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
        for io, snapshot in snapshots:
            self._rs_changed.append((io, snapshot))
            io.node.nmt.state = 'PRE-OPERATIONAL'
            self.sleep(io.settings.poll_interval_s)
            expected = (io.settings.target_cob_id, PDO_SYNCHRONOUS, (TARGET_POSITION_MAPPING,))
            self._write_mapping(io, *expected)
            if self._snapshot(io) != expected:
                raise TransportError(f'节点{io.settings.node_id} 同步目标 RPDO 回读不匹配')
        for io in self.axes + self.robstride_ios:
            # 保留实机现有SYNC周期。Kinco当前固件不接受1006=0，不能
            # 通过忽略回读差异来继续；PP共同触发只改变RPDO类型和必要的SYNC地址。
            original = io.read(SYNC_PERIOD_OBJECT)
            original_cob_id = io.read(SYNC_COB_OBJECT)
            self._sync_changed.append((io, original, original_cob_id))
            if original_cob_id != SYNC_COB_ID:
                io.write(SYNC_COB_OBJECT, SYNC_COB_ID)
                if io.read(SYNC_COB_OBJECT) != SYNC_COB_ID:
                    raise TransportError(f'节点 {io.settings.node_id} SYNC地址回读不匹配')
        feedback_axes = self.axes + self.robstride_ios
        self.feedback = CyclicFeedback(self.network, feedback_axes, feedback_axes,
                                       self.settings, lambda: self._cyclic_sync_active,
                                       remap_state_axes=self.robstride_ios)
        self.feedback.configure()

    def start_all(self):
        """启动Kinco；全部RobStride保持预操作态完成SDO准备，到共同触发前才切入运行态。"""
        super().start_all()

    def rs06_preoperational(self, io=None):
        """为指定RobStride IO进入预操作态并关闭同步门控，使SDO状态控制立即生效。

        RS06实机即使在NMT预操作态，传输类型1仍会让SDO控制字等待SYNC。
        因此先禁用目标RPDO，再改回异步254；不恢复原映射、不发SYNC或位置目标。
        仅在共同启动前重启同步映射，停止时保持通道禁用至确认全组静止。
        """
        io = self.rs06_io if io is None else io
        io.node.nmt.state = 'PRE-OPERATIONAL'
        self.sleep(io.settings.poll_interval_s)
        cob_id = io.read(self.comm_index, 1)
        io.write(self.comm_index, cob_id | PDO_DISABLED, 1)
        io.write(self.comm_index, PDO_ASYNCHRONOUS, 2)
        self.sleep(io.settings.poll_interval_s)
        if io.read(self.comm_index, 2) != PDO_ASYNCHRONOUS:
            raise TransportError(f'节点{io.settings.node_id} 停止/准备时关闭同步门控回读失败')

    def _start_rs06(self):
        """全部伴随轴已使能保持当前位置，再统一进入运行态并初始化同步接收。"""
        for io in self.robstride_ios:
            expected = (io.settings.target_cob_id, PDO_SYNCHRONOUS, (TARGET_POSITION_MAPPING,))
            self._write_mapping(io, *expected)
            if self._snapshot(io) != expected:
                raise TransportError(f'节点{io.settings.node_id} 共同触发前同步映射回读失败')
            io.start()
        self.sleep(max(io.settings.poll_interval_s for io in self.robstride_ios))
        # RS06 Revision0x500实机要求初始化SYNC；新型号共用EDS但仍须实机核对。
        # 全部伴随轴进入OP后统一初始化，此时Kinco仍Halt且没有装载任何新目标。
        self._send(SYNC_COB_ID, b'')
        self.sleep(max(io.settings.poll_interval_s for io in self.robstride_ios))

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
        timeout = min(io.settings.sdo_timeout_s for io in self.axes + self.robstride_ios)
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
            self.sync_period_s = min(periods) if periods else min(io.settings.poll_interval_s for io in self.robstride_ios)
            self.network.sync.start(self.sync_period_s)
            self._cyclic_sync_active = True

    def broadcast(self, controlword):
        """装载Kinco控制字及全部伴随目标后统一SYNC；停止时先逐轴停止RobStride，避免释放残留待处理目标。

        CAN 发帧后等待配置的发送余量再发高优先级 SYNC，避免 SYNC 抢先。
        余量依赖总线负载，必须实机核对；不是硬实时发送完成保证。
        """
        if any(peer is None for peer in self.robstride_peers) or len(self.robstride_peers) != len(self.robstride_ios):
            raise TransportError('RobStride 伴随控制器尚未全部绑定')
        stop_errors = []
        if controlword & CW_HALT:
            self._stop_cyclic_sync()
            self._motion_triggered = False
            for peer in self.robstride_peers:
                try:
                    peer.request_halt()
                except Exception as exc:
                    stop_errors.append(f'{peer.name}: {exc}')
        elif controlword & CW_NEW_SETPOINT:
            self._arm_kinco_sync(controlword)
            self._start_rs06()
            for peer in self.robstride_peers:
                self._send(peer.settings.target_cob_id, peer.target.to_bytes(4, 'little', signed=True))
        super().broadcast(controlword)
        if stop_errors:
            # 仍发送 Kinco Halt RPDO，但不以 SYNC 释放未确认停止的RobStride残留目标。
            # 控制器随后继续向每台Kinco和RobStride 分别发送 SDO 停止请求。
            raise TransportError('RobStride 停止请求失败，未发送 SYNC：' + '; '.join(stop_errors))
        self.sleep(max(io.settings.sync_settle_s for io in self.robstride_ios))
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
        failed_targets = []
        for io, snapshot in reversed(self._rs_changed):
            try:
                io.node.nmt.state = 'PRE-OPERATIONAL'
                self._write_mapping(io, *snapshot)
                if self._snapshot(io) != snapshot:
                    raise TransportError('原目标RPDO映射回读不匹配')
                io.start()
            except Exception as exc:
                failed_targets.append((io, snapshot))
                errors.append(f'节点{io.settings.node_id} 目标RPDO恢复：{exc}')
        self._rs_changed = failed_targets
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
            raise TransportError('混合多轴配置恢复失败：' + '; '.join(errors))

    def close(self):
        """释放连接前停止后台SYNC；驱动器停止仍由控制器负责。"""
        self._stop_cyclic_sync()
        if self.feedback is not None:
            self.feedback.detach()
        super().close()


class SimulatedFourAxisBus(SimulatedGroupBus):
    """混合轴协议模拟；保留三台Kinco共同帧，每台RobStride接收自己的绝对目标。"""

    def __init__(self, initial_positions=(0, 0, 0), rs06_position=0, extra_positions=()):
        """按inc初始化独立伴随轴；events兼容四轴，companion_events记录全部伴随目标。"""
        super().__init__(initial_positions)
        self.rs06_io = SimulatedRS06(rs06_position)
        self.extra_ios = [SimulatedRS06(position) for position in extra_positions]
        self.extra_peers = []
        self.peer = None
        self.events = []
        self.companion_events = []

    def broadcast(self, controlword):
        """模拟同步装载和释放；不模拟真实总线帧发送延迟或机械运动时间。"""
        if controlword & CW_HALT:
            errors = []
            for peer in [self.peer] + self.extra_peers:
                try:
                    peer.request_halt()
                except Exception as exc:
                    errors.append(f'{peer.name}: {exc}')
            super().broadcast(controlword)
            if errors:
                raise TransportError('; '.join(errors))
            return
        super().broadcast(controlword)
        if controlword & CW_NEW_SETPOINT and not controlword & CW_HALT:
            for peer in [self.peer] + self.extra_peers:
                peer.io.write(Object.TARGET_POSITION, peer.target)
            self.companion_events.append(tuple(peer.target for peer in [self.peer] + self.extra_peers))
            self.events.append(('sync_start', tuple(io.target for io in self.axes), self.peer.target))


class FourAxisController(ThreeAxisController):
    """补充全部轴的轮询观察时差；三台Kinco仍由基类统一装载、握手与联停。"""

    def run(self):
        """正常完成返回原三轴结果，主机观察时差同时计入所有有位移的伴随轴。"""
        starts = tuple(io.read(Object.ACTUAL_POSITION) for io in self.bus.axes)
        result = super().run()
        arrivals = [arrival for arrival, axis, start in zip(result.observed_arrival_times_s, result.plan.axes, starts)
                    if abs(axis.end_inc-start) > axis.axis.settings.position_tolerance_inc]
        arrivals.extend(peer.arrived_at for peer in self.peers if peer.has_motion)
        self.axis_observed_spread_s = max(arrivals) - min(arrivals) if arrivals else 0.0
        return result


class FourAxisReciprocationRunner(ReciprocationRunner):
    """混合多轴每程共同开始并按共同时间规划，全部到位和停稳后换向。"""

    def __init__(self, bus, settings, parameters, rs06,
                 clock=time.monotonic, sleep=time.sleep, sample_sink=None, extra_settings=()):
        """复用三轴端点逻辑，全部伴随轴保留独立恢复段与完整单程计数。"""
        validate_topology(settings, rs06, extra_settings)
        configs = (rs06,) + tuple(extra_settings)
        ios = [bus.rs06_io] + bus.extra_ios
        if len(configs) != len(ios):
            raise ValueError('RobStride配置数量与总线节点数量不同')
        self.peers = []
        for io, config in zip(ios, configs):
            enter = (lambda io=io: bus.rs06_preoperational(io)) if hasattr(bus, 'rs06_preoperational') else None
            self.peers.append(RS06Peer(io, config, clock, sleep, enter_preoperational=enter))
        self.peer = self.peers[0]
        bus.peer, bus.extra_peers = self.peer, self.peers[1:]
        self.axis_count = 3 + len(self.peers)
        self.coordination = []
        self.last_controller = None
        super().__init__(bus, settings, parameters, clock, sleep, sample_sink,
                         controller_factory=self._controller)

    def _controller(self, bus, settings, clock, sleep, poll_hook):
        """每程使用Kinco共享触发控制器，全部伴随轴纳入监视、到位等待和联停。"""
        if self.last_controller is not None:
            self._record_coordination()
        self.last_controller = FourAxisController(bus, settings, clock, sleep, poll_hook, peers=self.peers)
        self.last_controller.model_spread_s = self.model_spread_s
        return self.last_controller

    def _record_coordination(self):
        """分别保留整数模型时差和主机轮询时差；未完成程不伪造到位时间。"""
        controller = self.last_controller
        if controller is not None and hasattr(controller, 'axis_observed_spread_s'):
            self.coordination.append(dict(model_arrival_spread_s=controller.model_spread_s,
                observed_arrival_spread_s=controller.axis_observed_spread_s))
            self.coordination = self.coordination[-100:]
            LOG.info('%d轴量化模型到达时差 %.6fs，轮询观察到达时差 %.6fs（含通信和确认窗口）',
                     self.axis_count, controller.model_spread_s, controller.axis_observed_spread_s)
        self.last_controller = None

    def _feedback_axes(self):
        """按配置顺序输出全部轴反馈，RobStride速度使用各手册确认的0.1rpm单位。"""
        return super()._feedback_axes() + [(peer.io, peer.settings) for peer in self.peers]

    def _leg_settings(self, endpoints, starts=None, companion_starts=None):
        """按全部轴最慢轮廓规划共同秒数；inc端点不变，逐轴量化并校验时限。

        companion_starts仅供完整正反程只读校验注入端点；省略则读取实际位置。
        """
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
        targets = [peer.settings.far_position_inc if forward else peer.settings.origin_position_inc
                   for peer in self.peers]
        # 同一次规划复用同一位置快照，避免静止抖动造成最短时间矛盾。
        if companion_starts is None:
            companion_starts = [peer.check()['start_position_inc'] for peer in self.peers]
        minimums = [peer.plan(target, start_position_inc=start)
                    for peer, target, start in zip(self.peers, targets, companion_starts)]
        common = max([plan_group(settings, starts).duration_s] + minimums)
        if common:
            common += ABSOLUTE_POSITION_TIME_MARGIN_S
        settings = replace(settings, duration_s=common if common else None)
        plan = plan_group(settings, starts)
        durations = [peer.plan(target, common, start_position_inc=start)
                     for peer, target, start in zip(self.peers, targets, companion_starts)]
        moving = [axis.estimated_duration_s for axis, start in zip(plan.axes, starts) if axis.end_inc != start]
        moving.extend(duration for duration in durations if duration)
        self.model_spread_s = max(moving) - min(moving) if moving else 0
        LOG.info('%d轴共同规划 %.6fs，RobStride轮廓RPM=%s，量化模型到达时差 %.6fs',
                 self.axis_count, common, [peer.velocity_raw / 10 for peer in self.peers], self.model_spread_s)
        return settings

    def check(self):
        """只读校验全部端点及完整正反程，恢复段不能掩盖完整行程超时。"""
        checked = super().check()
        companion_checks = [peer.check() for peer in self.peers]
        for forward in (True, False):
            endpoints = checked['far_positions_inc'] if forward else checked['origin_positions_inc']
            starts = checked['origin_positions_inc'] if forward else checked['far_positions_inc']
            companion_starts = [peer.settings.origin_position_inc if forward else peer.settings.far_position_inc
                                for peer in self.peers]
            try:
                self._leg_settings(endpoints, starts, companion_starts)
            except ValueError as exc:
                raise ValueError(f'全部轴完整单程校验失败：{exc}') from exc
        forward = checked['effective_start_direction'] == 'forward'
        # 全组完整单程要求每个轴均从相反固定端点出发；各伴随轴仍另行独立计数。
        checked['partial_first_leg'] = checked['partial_first_leg'] or any(
            abs(row['start_position_inc'] - (peer.settings.origin_position_inc if forward
                else peer.settings.far_position_inc)) > peer.settings.position_tolerance_inc
            for peer, row in zip(self.peers, companion_checks))
        checked['axis_count'] = self.axis_count
        checked['rs06'] = companion_checks[0]
        checked['robstride_axes'] = [dict(row, model=getattr(peer.settings, 'model', 'RS06'))
                                     for peer, row in zip(self.peers, companion_checks)]
        self._leg_settings(checked['first_target_positions_inc'], checked['start_positions_inc'])
        for peer in self.peers:
            peer.preflight()
        return checked

    def run(self):
        """运行混合多轴并追加各伴随轴计数；异常时先请求全部停止再逐轴确认。"""
        try:
            return super().run()
        finally:
            self._record_coordination()
            self.report['rs06_completed_legs'] = self.peer.completed_legs
            self.report['rs06_completed_full_legs'] = self.peer.completed_full_legs
            self.report['rs06_completed_round_trips'] = self.peer.completed_full_legs // 2
            if self.axis_count == 4:
                self.report['four_axis_model_arrival_spread_s'] = getattr(self, 'model_spread_s', None)
                self.report['four_axis_recent_coordination'] = self.coordination
            self.report['axis_count'] = self.axis_count
            self.report['axis_model_arrival_spread_s'] = getattr(self, 'model_spread_s', None)
            self.report['axis_recent_coordination'] = self.coordination
            self.report['robstride_counts'] = [dict(node_id=peer.settings.node_id,
                model=getattr(peer.settings, 'model', 'RS06'), completed_legs=peer.completed_legs,
                completed_full_legs=peer.completed_full_legs,
                completed_round_trips=peer.completed_full_legs // 2) for peer in self.peers]
            self.report['hardware_validated'] = False


def main(argv=None):
    """独立四轴入口；--rs06-config 可覆盖第四轴配置，其余参数沿用三轴往返入口。"""
    from .reciprocate import main as reciprocate_main
    return reciprocate_main(['--config', 'config/four_axis.kinco.example.json',
                            '--rs06-config', 'config/rs06.example.json']
                            + list(sys.argv[1:] if argv is None else argv))


if __name__ == '__main__':
    raise SystemExit(main())
