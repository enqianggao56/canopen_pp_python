"""配置校验与单位换算；所有位置均为驱动器 inc，不推断机械传动比。"""

import json
import math
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Union

from .objects import INT32_MAX, INT32_MIN, UINT32_MAX


def positive(value: float, name: str) -> None:
    """要求数值有限且大于零；不满足时抛出 ValueError，无返回值。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是数值")
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须有限且大于 0")


def integer(value: int, low: int, high: int, name: str) -> None:
    """校验闭区间内的整数；拒绝布尔值，失败抛出 ValueError。"""
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} 必须是 [{low}, {high}] 范围内的整数")


@dataclass(frozen=True)
class Settings:
    """总线、换算和等待配置；时间单位秒，位置单位 inc。

    encoder_counts_per_rev 为通信换算分辨率；手册表 8-2 注明统一为 65536，
    不能根据编码器标注的位数直接推导替换。仍需确认实机固件适用性。
    hardware_verified 表示操作者已核对型号、固件、EDS、单位及停止方式。
    """

    eds: str
    encoder_counts_per_rev: int
    min_position_inc: int
    max_position_inc: int
    max_velocity_rpm: float
    max_acceleration_rps2: float
    expected_vendor_id: int
    expected_product_code: int
    interface: str = "socketcan"
    channel: Union[str, int] = "can0"
    node_id: int = 1
    bitrate: int = 1000000
    sdo_timeout_s: float = 0.3
    state_timeout_s: float = 3.0
    motion_timeout_s: float = 30.0
    poll_interval_s: float = 0.02
    position_tolerance_inc: int = 10
    zero_velocity_tolerance_rpm: float = 0.0
    standstill_duration_s: float = 0.05
    hardware_verified: bool = False
    absolute_reference_verified: bool = False

    def __post_init__(self) -> None:
        """创建时校验类型与范围；无效配置抛出 ValueError。"""
        integer(self.encoder_counts_per_rev, 1, UINT32_MAX, "编码器分辨率")
        integer(self.node_id, 1, 127, "节点 ID")
        integer(self.bitrate, 1, 1000000, "CAN 波特率")
        integer(self.min_position_inc, INT32_MIN, INT32_MAX, "最小位置")
        integer(self.max_position_inc, INT32_MIN, INT32_MAX, "最大位置")
        if self.min_position_inc >= self.max_position_inc:
            raise ValueError("最小位置必须小于最大位置")
        integer(self.position_tolerance_inc, 0, INT32_MAX, "到位容差")
        for name in ("expected_vendor_id", "expected_product_code"):
            integer(getattr(self, name), 0, UINT32_MAX, name)
        for name in ("max_velocity_rpm", "max_acceleration_rps2", "sdo_timeout_s",
                     "state_timeout_s", "motion_timeout_s", "poll_interval_s",
                     "standstill_duration_s"):
            positive(getattr(self, name), name)
        tolerance = self.zero_velocity_tolerance_rpm
        if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
                or not math.isfinite(tolerance) or tolerance < 0):
            raise ValueError("zero_velocity_tolerance_rpm 必须有限且非负")
        if self.standstill_duration_s >= min(self.state_timeout_s, self.motion_timeout_s):
            raise ValueError("静止确认时长必须小于状态和运动超时")
        for name in ("hardware_verified", "absolute_reference_verified"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name} 必须是 JSON 布尔值")
        if not isinstance(self.eds, str) or not self.eds:
            raise ValueError("eds 必须是文件路径")
        if not isinstance(self.interface, str) or not self.interface:
            raise ValueError("interface 必须是接口名称")
        if (type(self.channel) not in (str, int)
                or (isinstance(self.channel, str) and not self.channel)):
            raise ValueError("channel 必须是非空字符串或整数")

    @classmethod
    def load(cls, path: Union[str, Path]) -> "Settings":
        """读取 JSON 配置并返回 Settings；EDS 路径相对于配置文件解析。

        文件、JSON 或字段错误保留为 OSError、ValueError、TypeError。
        """
        path = Path(path).resolve()
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if not isinstance(data, dict):
            raise ValueError("配置顶层必须是 JSON 对象")
        unknown = set(data) - {field.name for field in fields(cls)}
        if unknown:
            raise ValueError(f"未知配置项：{sorted(unknown)}")
        settings = cls(**data)
        # 在类型校验后解析，避免将 null 或数字当成路径。
        return replace(settings, eds=str((path.parent / settings.eds).resolve()))

    def velocity_raw(self, rpm: float) -> int:
        """把 rpm 转成 0x6081 的 uint32；依据手册 §7.3，向下取整。

        DEC = rpm * 512 * 编码器分辨率 / 1875；溢出或量化为零抛 ValueError。
        """
        positive(rpm, "速度 rpm")
        return self._raw(rpm * 512 * self.encoder_counts_per_rev / 1875)

    def feedback_velocity_rpm(self, raw: int) -> float:
        """按 Kinco 手册 §7.2 将 0x606C 有符号原始速度转换为 rpm。"""
        return raw * 1875 / (512 * self.encoder_counts_per_rev)

    def acceleration_raw(self, rps2: float) -> int:
        """把 rps/s 转成 0x6083/0x6084 的 uint32，向下取整。

        手册 §7.3：DEC = rps/s * 65536 * 编码器分辨率 / 4000000。
        溢出或量化为零抛 ValueError。
        """
        positive(rps2, "加减速度 rps/s")
        return self._raw(rps2 * 65536 * self.encoder_counts_per_rev / 4000000)

    @staticmethod
    def _raw(value: float) -> int:
        """将有限的正换算结果截断为 uint32；无效结果抛 ValueError。"""
        if not math.isfinite(value) or not 1 <= value <= UINT32_MAX:
            raise ValueError("换算结果为零或超出 uint32，请核对分辨率与运动参数")
        return int(value)


@dataclass(frozen=True)
class Motion:
    """单次运动：target_inc 为绝对坐标或相对位移，其他参数使用物理单位。"""

    target_inc: int
    velocity_rpm: float
    acceleration_rps2: float
    deceleration_rps2: float
    relative: bool = True

    def validate(self, settings: Settings, start_inc: int) -> int:
        """校验运动并返回预期绝对终点 inc；越界或参数错误抛 ValueError。"""
        integer(self.target_inc, INT32_MIN, INT32_MAX, "目标位置/位移")
        integer(start_inc, INT32_MIN, INT32_MAX, "实际位置")
        if type(self.relative) is not bool:
            raise ValueError("relative 必须是布尔值")
        end = start_inc + self.target_inc if self.relative else self.target_inc
        for position in (start_inc, end):
            if not settings.min_position_inc <= position <= settings.max_position_inc:
                raise ValueError(f"位置 {position} inc 超出配置的软件范围")
        if not self.relative and not settings.absolute_reference_verified:
            raise ValueError("绝对定位前必须确认坐标原点并设置 absolute_reference_verified")
        for value, limit, name in (
            (self.velocity_rpm, settings.max_velocity_rpm, "速度 rpm"),
            (self.acceleration_rps2, settings.max_acceleration_rps2, "加速度 rps/s"),
            (self.deceleration_rps2, settings.max_acceleration_rps2, "减速度 rps/s"),
        ):
            positive(value, name)
            if value > limit:
                raise ValueError(f"{name} 超过配置上限 {limit}")
        settings.velocity_raw(self.velocity_rpm)
        settings.acceleration_raw(self.acceleration_rps2)
        settings.acceleration_raw(self.deceleration_rps2)
        return end
