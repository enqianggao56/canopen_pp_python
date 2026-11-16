"""运动期Kinco周期PDO反馈；配置/停止后仍用SDO，避免TPDO与密集SDO应答竞争。

TPDO1必须已有6041(16)+实际位置(32)+6061(8)布局；临时TPDO2映射606C(32)。
依据Kinco手册§9.3.3，速度及位置仍使用设备原始单位，不由模型估算替代反馈。
"""

import time

from .group_transport import CanopenGroupBus, PDO_DISABLED
from .objects import Object
from .transport import TransportError


class CyclicFeedback:
    """共享网络中的三轴TPDO缓存；失效即报错，不在运动期降级到高频SDO轮询。"""

    def __init__(self, network, axes, all_axes, settings, active, clock=time.monotonic, sleep=time.sleep):
        """注入网络、三轴IO、总线全部IO、组配置及运动期启用判定；不访问设备。"""
        self.network, self.axes, self.all_axes = network, axes, all_axes
        self.active, self.clock, self.sleep = active, clock, sleep
        self.cache = {}
        self.changed, self.subscriptions = [], []
        self.mapper = CanopenGroupBus(settings)
        self.mapper.comm_index, self.mapper.map_index = 0x1801, 0x1A01

    def _provider(self, io):
        """创建特定节点的反馈读取器，返回原始整数；非运动期返回None以走SDO。"""
        node = io.settings.node_id
        position_index = io.settings.actual_position_index
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
        """先检查布局/COB-ID，再暂存并配置三轴TPDO2速度；失败仍保留已改动快照。"""
        if self.changed or self.subscriptions:
            raise TransportError('存在未清理的周期反馈配置')
        wanted = {0x280 + io.settings.node_id for io in self.axes}
        for io in self.all_axes:
            for index in io.node.object_dictionary:
                if not (0x1400 <= index <= 0x1407 or 0x1800 <= index <= 0x1807):
                    continue
                if index == 0x1801 and io in self.axes:
                    continue
                cob_id = io.read(index, 1)
                if not cob_id & PDO_DISABLED and cob_id & 0x1FFFFFFF in wanted:
                    raise TransportError(f'节点{io.settings.node_id} 其他PDO占用速度TPDO预留COB-ID')
        originals = []
        state_cobs = []
        for io in self.axes:
            expected = [(int(Object.STATUSWORD) << 16) | 16,
                        (io.settings.actual_position_index << 16) | 32,
                        (int(Object.MODE_DISPLAY) << 16) | 8]
            count = io.read(0x1A00, 0)
            if count != 3 or [io.read(0x1A00, sub) for sub in (1, 2, 3)] != expected:
                raise TransportError(f'节点{io.settings.node_id} TPDO1须为6041+实际位置+6061七字节布局')
            cob_id = io.read(0x1800, 1)
            if cob_id & PDO_DISABLED or cob_id & (1 << 29) or io.read(0x1800, 2) != 1:
                raise TransportError(f'节点{io.settings.node_id} TPDO1须为启用的标准帧同步类型1')
            state_cobs.append(cob_id & 0x7FF)
            originals.append((io, self.mapper._snapshot(io), io.read(0x1801, 3)))
        for (io, snapshot, inhibit), state_cob in zip(originals, state_cobs):
            node = io.settings.node_id
            self.cache[node] = {}
            def state(can_id, data, timestamp, node=node):
                """按已校验的七字节映射保存状态、位置、模式和接收时刻。"""
                if len(data) >= 7:
                    self.cache[node].update(status=int.from_bytes(data[:2], 'little'),
                        position=int.from_bytes(data[2:6], 'little', signed=True),
                        mode=int.from_bytes(data[6:7], 'little', signed=True), state_time=self.clock())
            def velocity(can_id, data, timestamp, node=node):
                """保存606C原始有符号速度，保持手册换算与独立速度反馈来源。"""
                if len(data) >= 4:
                    self.cache[node].update(velocity=int.from_bytes(data[:4], 'little', signed=True),
                                            velocity_time=self.clock())
            speed_cob = 0x280 + node
            for cob_id, callback in ((state_cob, state), (speed_cob, velocity)):
                self.network.subscribe(cob_id, callback)
                self.subscriptions.append((cob_id, callback))
            self.changed.append((io, snapshot, inhibit))
            io.node.nmt.state = 'PRE-OPERATIONAL'
            io.write(0x1801, snapshot[0] | PDO_DISABLED, 1)
            io.write(0x1801, 0, 3)
            expected = (speed_cob, 1, ((int(Object.ACTUAL_VELOCITY) << 16) | 32,))
            self.mapper._write_mapping(io, *expected)
            if self.mapper._snapshot(io) != expected or io.read(0x1801, 3) != 0:
                raise TransportError(f'节点{node} 速度TPDO2回读不一致')
            io.feedback_provider = self._provider(io)

    def detach(self):
        """取消本对象的订阅及读取钩子，不修改设备；允许在连接关闭时调用。"""
        for io in self.axes:
            io.feedback_provider = None
        for cob_id, callback in self.subscriptions:
            self.network.unsubscribe(cob_id, callback)
        self.subscriptions = []

    def restore(self):
        """全组已停稳后逐轴恢复原速度TPDO2及禁止时间，失败也继续其余轴。"""
        errors, failed = [], []
        self.detach()
        for io, snapshot, inhibit in reversed(self.changed):
            try:
                io.node.nmt.state = 'PRE-OPERATIONAL'
                self.mapper._write_mapping(io, *snapshot)
                io.write(0x1801, inhibit, 3)
                if self.mapper._snapshot(io) != snapshot or io.read(0x1801, 3) != inhibit:
                    raise TransportError('原速度TPDO回读不一致')
                io.start()
            except Exception as exc:
                failed.append((io, snapshot, inhibit))
                errors.append(f'节点{io.settings.node_id} 速度TPDO恢复失败：{exc}')
        self.changed = failed
        if errors:
            raise TransportError('; '.join(errors))
