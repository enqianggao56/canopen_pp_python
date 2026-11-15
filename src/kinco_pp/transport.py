"""CANopen SDO 适配层；使用真实 EDS 的数据类型读写，不自动重试运动写入。"""

import logging
from typing import Protocol

from .model import Settings
from .objects import Object

LOG = logging.getLogger(__name__)


class TransportError(RuntimeError):
    """总线连接、SDO 访问或身份核对失败；消息包含对象索引。"""


class DriveIO(Protocol):
    """控制器依赖的最小接口；可由真实总线或模拟驱动器实现。"""

    def read(self, index: int, subindex: int = 0) -> int:
        """读取整数对象；失败抛 TransportError，返回值使用设备原始单位。"""
        ...

    def write(self, index: int, value: int, subindex: int = 0) -> None:
        """写入整数对象；失败抛 TransportError，无返回值。"""
        ...


class CanopenTransport:
    """管理单节点 CANopen 网络；构造时不连接，open 后方可访问对象。"""

    def __init__(self, settings: Settings):
        """保存配置；settings 描述接口、节点和 SDO 超时，无返回值。"""
        self.settings = settings
        self.network = None
        self.node = None

    def open(self) -> None:
        """加载 EDS 并连接总线；不发送使能、NMT 启动或运动指令。

        依赖、EDS 或接口异常包装为 TransportError，并释放已连接资源。
        """
        try:
            import canopen
            self.network = canopen.Network()
            self.node = self.network.add_node(self.settings.node_id, self.settings.eds)
            # SDO 往返次数固定为一次：响应丢失时不自动重发相对运动触发。
            self.node.sdo.RESPONSE_TIMEOUT = self.settings.sdo_timeout_s
            self.node.sdo.MAX_RETRIES = 1
            self.network.connect(
                interface=self.settings.interface, channel=self.settings.channel,
                bitrate=self.settings.bitrate,
            )
        except Exception as exc:
            self.close()
            raise TransportError(f"连接 CANopen 失败：{exc}") from exc

    def _variable(self, index: int, subindex: int):
        """按 EDS 查找变量；未连接或对象缺失抛 TransportError，返回 SdoVariable。"""
        if self.node is None:
            raise TransportError("CANopen 尚未连接")
        variable = self.node.sdo[index]
        # EDS 简单变量直接访问；记录/数组使用子索引。
        if hasattr(variable, "__getitem__"):
            return variable[subindex]
        if subindex != 0:
            raise TransportError(f"0x{index:04X} 不是记录，不能访问子索引 {subindex}")
        return variable

    def read(self, index: int, subindex: int = 0) -> int:
        """通过 SDO 返回整数原始值；通信、类型或对象错误抛 TransportError。"""
        try:
            self.network.check()
            value = self._variable(index, subindex).raw
            if not isinstance(value, int):
                raise ValueError("对象不是整数类型")
            LOG.debug("SDO 读 0x%04X:%02X = %d", index, subindex, value)
            return value
        except Exception as exc:
            raise TransportError(f"SDO 读 0x{index:04X}:{subindex:02X} 失败：{exc}") from exc

    def write(self, index: int, value: int, subindex: int = 0) -> None:
        """以 EDS 类型编码并写入原始值；失败抛 TransportError，不自动重试。"""
        try:
            self.network.check()
            LOG.debug("SDO 写 0x%04X:%02X = %d", index, subindex, value)
            self._variable(index, subindex).raw = value
        except Exception as exc:
            raise TransportError(f"SDO 写 0x{index:04X}:{subindex:02X} 失败：{exc}") from exc

    def verify_identity(self) -> dict:
        """核对配置、EDS 和 0x1018 的厂商/产品标识，返回实际身份信息。

        型号及固件适用性仍需人工核对；本方法不会根据相近型号绕过不一致。
        不匹配抛 TransportError，且不发送控制指令。
        """
        entries = self.read(Object.IDENTITY, 0)
        if entries < 2:
            raise TransportError("0x1018 未提供核对所需的厂商和产品标识")
        # 实机 0x1018:00=3，未实现可选序列号；按实际条目数读取，
        # 不因该可选字段缺失而跳过必需的厂商/产品一致性检查。
        observed = {
            "vendor_id": self.read(Object.IDENTITY, 1),
            "product_code": self.read(Object.IDENTITY, 2),
            "revision": self.read(Object.IDENTITY, 3) if entries >= 3 else None,
            "serial_number": self.read(Object.IDENTITY, 4) if entries >= 4 else None,
        }
        for subindex, name, expected in (
            (1, "vendor_id", self.settings.expected_vendor_id),
            (2, "product_code", self.settings.expected_product_code),
        ):
            eds_value = self.node.object_dictionary[Object.IDENTITY][subindex].default
            if observed[name] != expected or eds_value != expected:
                raise TransportError(
                    f"身份不匹配 {name}：设备={observed[name]}，配置={expected}，EDS={eds_value}"
                )
        return observed

    def start(self) -> None:
        """仅向配置的节点发送 NMT 启动；失败抛 TransportError，无返回值。"""
        try:
            self.node.nmt.state = "OPERATIONAL"
        except Exception as exc:
            raise TransportError(f"节点 NMT 启动失败：{exc}") from exc

    def close(self) -> None:
        """释放 CAN 连接，无返回值；断开连接不会自动停止或去使能驱动器。"""
        if self.network is not None and self.network.bus is not None:
            try:
                self.network.disconnect()
            finally:
                self.node = None
