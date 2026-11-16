"""运动期Kinco/RobStride周期PDO反馈；配置/停止后仍用SDO，避免TPDO与密集SDO应答竞争。

Kinco校验既有TPDO1，RobStride临时配置TPDO1：6041(16)+实际位置(32)+6061(8)。
各轴临时TPDO2映射606C(32)，类型1随每次SYNC反馈。依据Kinco手册§9.3.3、
RS06/RS00/EL05手册§5.2/§5.4及共用RS06.eds；位置/速度保留各厂家的原始单位。
"""

import time

from .group_transport import CanopenGroupBus, PDO_DISABLED
from .objects import Object
from .transport import TransportError


class CyclicFeedback:
    """共享网络中的多轴TPDO缓存；失效即报错，不在运动期降级到高频SDO轮询。"""

    def __init__(self, network, axes, all_axes, settings, active, clock=time.monotonic,
                 sleep=time.sleep, remap_state_axes=()):
        """注入反馈轴、总线全部轴及运动期判定；clock返回秒，不访问设备。

        remap_state_axes指定需临时重映射TPDO1的轴（RS06/RS00/EL05）；其余轴
        只校验既有TPDO1。全部反馈轴均临时配置TPDO2，异常由调用方联停并恢复。
        """
        self.network, self.axes, self.all_axes = network, axes, all_axes
        self.active, self.clock, self.sleep = active, clock, sleep
        self.cache = {}
        self.changed, self.subscriptions = [], []
        self.remap_state_axes = tuple(remap_state_axes)
        self.mappers = {}
        # CiA 301：1800/1801为TPDO通信参数，1A00/1A01为对应映射。
        for comm_index in (0x1800, 0x1801):
            mapper = CanopenGroupBus(settings)
            mapper.comm_index, mapper.map_index = comm_index, comm_index + 0x200
            self.mappers[comm_index] = mapper

    def _provider(self, io):
        """创建特定节点的反馈读取器，返回原始整数；非运动期返回None以走SDO。"""
        node = io.settings.node_id
        position_index = getattr(io.settings, 'actual_position_index', int(Object.ACTUAL_POSITION))
        fields = {int(Object.STATUSWORD): ('status', 'state_time'),
                  position_index: ('position', 'state_time'),
                  int(Object.MODE_DISPLAY): ('mode', 'state_time'),
                  int(Object.ACTUAL_VELOCITY): ('velocity', 'velocity_time')}
        def read(index, subindex):
            """等待首帧或检查最新帧年龄；超出本节点通信超时抛TransportError。"""
            if not self.active() or subindex != 0 or int(index) not in fields:
                return None
            value_field, time_field = fields[int(index)]
            deadline = self.clock() + io.settings.sdo_timeout_s
            while time_field not in self.cache[node]:
                if self.clock() >= deadline:
                    raise TransportError(f'节点{node} 等待首帧TPDO反馈超时')
                self.sleep(0.001)
            entry = self.cache[node]
            if self.clock() - entry[time_field] > io.settings.sdo_timeout_s:
                raise TransportError(f'节点{node} TPDO反馈过期（{time_field}）')
            return entry[value_field]
        return read

    def configure(self):
        """先读取全组快照和检查COB-ID，再配置TPDO；失败保留快照供停稳后恢复。

        RobStride的TPDO1/2使用0x180+节点号、0x280+节点号；清零禁止时间、事件定时器
        和SYNC起始值，保证无计数器SYNC可触发反馈，退出时恢复这些参数。
        """
        if self.changed or self.subscriptions:
            raise TransportError('存在未清理的周期反馈配置')
        plans, owners, state_cobs = [], {}, {}
        for io in self.axes:
            node = io.settings.node_id
            position_index = getattr(io.settings, 'actual_position_index', int(Object.ACTUAL_POSITION))
            state_mapping = ((int(Object.STATUSWORD) << 16) | 16,
                             (position_index << 16) | 32,
                             (int(Object.MODE_DISPLAY) << 16) | 8)
            state_snapshot = self.mappers[0x1800]._snapshot(io)
            if io in self.remap_state_axes:
                state_cob = 0x180 + node
            else:
                cob_id, transmission, mappings = state_snapshot
                if mappings != state_mapping:
                    raise TransportError(f'节点{node} TPDO1须为6041+实际位置+6061七字节布局')
                if cob_id & PDO_DISABLED or cob_id & (1 << 29) or transmission != 1:
                    raise TransportError(f'节点{node} TPDO1须为启用的标准帧同步类型1')
                state_cob = cob_id & 0x7FF
            state_cobs[node] = state_cob
            for comm_index, cob_id in ((0x1800, state_cob), (0x1801, 0x280 + node)):
                if cob_id in owners:
                    raise TransportError(f'节点{node} 反馈TPDO COB-ID重复：0x{cob_id:X}')
                owners[cob_id] = (io, comm_index)
                if comm_index == 0x1800 and io not in self.remap_state_axes:
                    continue
                mapper = self.mappers[comm_index]
                snapshot = state_snapshot if comm_index == 0x1800 else mapper._snapshot(io)
                subindices = (3, 5, 6) if io in self.remap_state_axes else (3,)
                parameters = {sub: io.read(comm_index, sub) for sub in subindices}
                mapping = state_mapping if comm_index == 0x1800 else (
                    (int(Object.ACTUAL_VELOCITY) << 16) | 32,)
                plans.append((io, comm_index, snapshot, parameters, (cob_id, 1, mapping)))
        # 将被重映射的槽位可释放旧ID；其余启用PDO不能与任何反馈通道共用ID。
        replaced = {(io, comm_index) for io, comm_index, _, _, _ in plans}
        for io in self.all_axes:
            for index in io.node.object_dictionary:
                if not (0x1400 <= index <= 0x1407 or 0x1800 <= index <= 0x1807):
                    continue
                if (io, index) in replaced:
                    continue
                cob_id = io.read(index, 1)
                owner = owners.get(cob_id & 0x1FFFFFFF)
                if not cob_id & PDO_DISABLED and owner is not None and owner != (io, index):
                    raise TransportError(f'节点{io.settings.node_id} 其他PDO占用反馈TPDO预留COB-ID')
        for io in self.axes:
            node = io.settings.node_id
            self.cache[node] = {}
            def state(can_id, data, timestamp, node=node):
                """仅接受七字节映射，按小端保存无符号状态及有符号位置、模式。"""
                if len(data) == 7:
                    self.cache[node].update(status=int.from_bytes(data[:2], 'little'),
                        position=int.from_bytes(data[2:6], 'little', signed=True),
                        mode=int.from_bytes(data[6:7], 'little', signed=True), state_time=self.clock())
            def velocity(can_id, data, timestamp, node=node):
                """仅接受四字节606C有符号原始速度，换算仍由各轴配置负责。"""
                if len(data) == 4:
                    self.cache[node].update(velocity=int.from_bytes(data, 'little', signed=True),
                                            velocity_time=self.clock())
            for cob_id, callback in ((state_cobs[node], state), (0x280 + node, velocity)):
                self.network.subscribe(cob_id, callback)
                self.subscriptions.append((cob_id, callback))
        for io, comm_index, snapshot, parameters, expected in plans:
            # 在首次写入前登记，覆盖设备部分接受配置后SDO失败的回滚场景。
            self.changed.append((io, comm_index, snapshot, parameters))
            mapper = self.mappers[comm_index]
            io.node.nmt.state = 'PRE-OPERATIONAL'
            io.write(comm_index, snapshot[0] | PDO_DISABLED, 1)
            for sub in parameters:
                io.write(comm_index, 0, sub)
            mapper._write_mapping(io, *expected)
            if mapper._snapshot(io) != expected or any(io.read(comm_index, sub) != 0 for sub in parameters):
                raise TransportError(f'节点{io.settings.node_id} TPDO{comm_index - 0x1800 + 1}回读不一致')
        for io in self.axes:
            io.feedback_provider = self._provider(io)

    def reset(self):
        """共同运动SYNC前清除准备阶段的缓存，首轮读取必须等新的反馈帧。"""
        for io in self.axes:
            self.cache[io.settings.node_id] = {}

    def detach(self):
        """取消本对象的订阅及读取钩子，不修改设备；允许在连接关闭时调用。"""
        for io in self.axes:
            io.feedback_provider = None
        for cob_id, callback in self.subscriptions:
            self.network.unsubscribe(cob_id, callback)
        self.subscriptions = []

    def restore(self):
        """全组已停稳后恢复全部临时TPDO映射及通信参数，失败也继续其余槽位。"""
        errors, failed = [], []
        self.detach()
        for io, comm_index, snapshot, parameters in reversed(self.changed):
            try:
                io.node.nmt.state = 'PRE-OPERATIONAL'
                mapper = self.mappers[comm_index]
                mapper._write_mapping(io, *snapshot)
                for sub, value in parameters.items():
                    io.write(comm_index, value, sub)
                if mapper._snapshot(io) != snapshot or any(
                        io.read(comm_index, sub) != value for sub, value in parameters.items()):
                    raise TransportError('原TPDO配置回读不一致')
                io.start()
            except Exception as exc:
                failed.append((io, comm_index, snapshot, parameters))
                errors.append(f'节点{io.settings.node_id} TPDO{comm_index - 0x1800 + 1}恢复失败：{exc}')
        self.changed = failed
        if errors:
            raise TransportError('; '.join(errors))
