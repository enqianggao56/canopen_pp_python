"""共享 CAN 网络和 RPDO 控制字广播；依据手册 §9.3.3 与 CANopen PDO 接口。

三轴的同一 RPDO 槽位临时映射到同一 COB-ID、0x6040:00（16 bit）。
传输类型 254 使三个从站响应同一帧，不依赖逐轴 SDO 启动或 Python 线程调度。
"""

from .group_model import GroupSettings
from .model import integer
from .objects import Object
from .simulation import SimulatedDrive
from .transport import CanopenTransport, TransportError

PDO_DISABLED = 1 << 31
PDO_CONTROLWORD_MAPPING = (int(Object.CONTROLWORD) << 16) | 16
PDO_ASYNCHRONOUS = 254


class CanopenGroupBus:
    """三轴共用一个 CANopen Network；拥有连接与临时 PDO 配置快照。"""

    def __init__(self, settings: GroupSettings):
        """保存组配置；构造时不访问总线、不修改设备。"""
        self.settings = settings
        self.network = None
        self.axes = []
        self._snapshots = []
        self._changed = []
        self.configured = False
        self.comm_index = 0x1400 + settings.rpdo_number - 1
        self.map_index = 0x1600 + settings.rpdo_number - 1

    def open(self) -> None:
        """加载三个 EDS 并一次连接总线；失败抛 TransportError，不写控制对象。"""
        import canopen
        try:
            self.network = canopen.Network()
            for axis in self.settings.axes:
                transport = CanopenTransport(axis.settings, network=self.network)
                self.axes.append(transport)
                transport.open()
            first = self.settings.axes[0].settings
            self.network.connect(interface=first.interface, channel=first.channel, bitrate=first.bitrate)
        except Exception as exc:
            self.close()
            raise TransportError(f"三轴总线连接失败：{exc}") from exc

    def verify_identities(self):
        """返回三轴身份；任何一轴不匹配立即失败，不发送 NMT 或使能指令。"""
        identities = []
        for config, axis in zip(self.settings.axes, self.axes):
            try:
                identities.append(axis.verify_identity())
            except Exception as exc:
                raise TransportError(
                    f"轴 {config.name}/节点 {config.settings.node_id} 身份核对失败：{exc}"
                ) from exc
        return tuple(identities)

    def _snapshot(self, axis):
        """读取待修改 RPDO 的 COB-ID、传输类型及映射，返回可恢复的完整快照。"""
        count = axis.read(self.map_index, 0)
        integer(count, 0, 8, "原 RPDO 映射条目数")
        return (axis.read(self.comm_index, 1), axis.read(self.comm_index, 2),
                tuple(axis.read(self.map_index, sub) for sub in range(1, count + 1)))

    def _write_mapping(self, axis, cob_id, transmission, mappings) -> None:
        """先禁用 RPDO 再更改映射，最后恢复目标有效位；通信错误不自动重试。"""
        current = axis.read(self.comm_index, 1)
        axis.write(self.comm_index, current | PDO_DISABLED, 1)
        axis.write(self.map_index, 0, 0)
        for sub, mapping in enumerate(mappings, 1):
            axis.write(self.map_index, mapping, sub)
        axis.write(self.map_index, len(mappings), 0)
        axis.write(self.comm_index, transmission, 2)
        axis.write(self.comm_index, cob_id, 1)

    def configure(self) -> None:
        """核对组内 PDO 冲突，暂存原配置，在预操作状态重映射共享控制字。

        所有快照和冲突检查均在第一次写入前完成。部分配置失败时，调用方
        仍必须调用 restore，恢复已经开始修改的轴。不会写入 EEPROM。
        """
        if self._changed:
            raise TransportError("存在尚未恢复的 RPDO 配置，不能重复配置")
        if not self.settings.pdo_configuration_verified:
            raise TransportError("请先确认共享 COB-ID 和 RPDO 槽位可用，再设置 pdo_configuration_verified=true")
        self._snapshots = [self._snapshot(axis) for axis in self.axes]
        for axis in self.axes:
            for index in axis.node.object_dictionary:
                if (0x1400 <= index <= 0x1407 or 0x1800 <= index <= 0x1807) and index != self.comm_index:
                    cob_id = axis.read(index, 1)
                    if not cob_id & PDO_DISABLED and cob_id & 0x1FFFFFFF == self.settings.shared_cob_id:
                        raise TransportError(f"节点 {axis.settings.node_id} 已有其他 PDO 使用共享 COB-ID")
        expected = (self.settings.shared_cob_id, PDO_ASYNCHRONOUS, (PDO_CONTROLWORD_MAPPING,))
        for axis, snapshot in zip(self.axes, self._snapshots):
            self._changed.append((axis, snapshot))
            axis.node.nmt.state = "PRE-OPERATIONAL"
            self._write_mapping(axis, *expected)
            if self._snapshot(axis) != expected:
                raise TransportError(f"节点 {axis.settings.node_id} RPDO 回读不匹配")
        self.configured = True

    def start_all(self) -> None:
        """逐节点启动 NMT；此时三轴仍处于 Halt，NMT 启动不作为运动同步触发。"""
        for axis in self.axes:
            axis.start()

    def broadcast(self, controlword: int) -> None:
        """以一条 2 字节 CAN 帧广播控制字；发送失败抛 TransportError，不重发。"""
        import can
        if not self.configured:
            raise TransportError("共享 RPDO 尚未完整配置")
        integer(controlword, 0, 0xFFFF, "控制字")
        try:
            self.network.check()
            message = can.Message(arbitration_id=self.settings.shared_cob_id,
                                  is_extended_id=False, data=controlword.to_bytes(2, "little"))
            timeout = min(a.settings.sdo_timeout_s for a in self.settings.axes)
            with self.network.send_lock:
                self.network.bus.send(message, timeout=timeout)
        except Exception as exc:
            raise TransportError(f"三轴 RPDO 广播失败：{exc}") from exc

    def restore(self) -> None:
        """在已停稳后恢复所有已改动 RPDO；逐轴尝试，汇总失败，不中途跳过其他轴。"""
        errors = []
        failed = []
        self.configured = False
        for axis, snapshot in reversed(self._changed):
            try:
                axis.node.nmt.state = "PRE-OPERATIONAL"
                self._write_mapping(axis, *snapshot)
                if self._snapshot(axis) != snapshot:
                    raise TransportError("原 RPDO 配置回读不一致")
                axis.start()
            except Exception as exc:
                failed.append((axis, snapshot))
                errors.append(f"节点 {axis.settings.node_id}: {exc}")
        self._changed = failed
        if errors:
            raise TransportError("RPDO 恢复失败：" + "; ".join(errors))

    def close(self) -> None:
        """仅释放共享总线；不自动操作电机，调用方需先停止并恢复 RPDO。"""
        if self.network is not None and self.network.bus is not None:
            self.network.disconnect()
        self.axes = []


class SimulatedGroupBus:
    """三个独立模拟轴，共用一次逻辑帧广播；不模拟 CAN 时序或机械动力学。"""

    def __init__(self, initial_positions=(0, 0, 0)):
        """指定三轴初始位置 inc，记录广播帧以供测试检查是否重复触发。"""
        self.axes = [SimulatedDrive(position) for position in initial_positions]
        self.frames = []
        self.configured = False
        self.restored = False

    def configure(self) -> None:
        """启用模拟映射，不进行通信。"""
        self.configured = True

    def start_all(self) -> None:
        """模拟节点就绪；无实际 CAN 操作。"""

    def broadcast(self, controlword: int) -> None:
        """记录一次组帧，并将同一控制字交给三个模拟驱动器。"""
        if not self.configured:
            raise TransportError("模拟 RPDO 尚未配置")
        self.frames.append(controlword)
        for axis in self.axes:
            axis.write(Object.CONTROLWORD, controlword)

    def restore(self) -> None:
        """标记模拟映射恢复，不修改模拟驱动器的使能状态。"""
        self.configured = False
        self.restored = True
