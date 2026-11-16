"""三轴配置和时间缩放规划；共同启动，各轴保持独立 PP 位置闭环。"""

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional, Tuple

from .model import Motion, Settings, integer, positive


@dataclass(frozen=True)
class AxisConfig:
    """单轴配置；origin_position_inc 为可选往返固定起点，不修改驱动器坐标零点。"""

    name: str
    settings: Settings
    motion: Motion
    origin_position_inc: Optional[int] = None


@dataclass(frozen=True)
class GroupSettings:
    """同一总线的三个轴，以及预留的共享 RPDO 控制字通道。

    pdo_configuration_verified 表示操作者已确认所选 RPDO 槽位可临时重映射，
    shared_cob_id 未被组外设备或其他主站使用；不表示已有三轴实机验证。
    duration_s 为可选的共同运动时长，不能短于任何轴的最短轮廓。
    """

    axes: Tuple[AxisConfig, ...]
    rpdo_number: int = 4
    shared_cob_id: int = 0x500
    pdo_configuration_verified: bool = False
    duration_s: Optional[float] = None

    def __post_init__(self) -> None:
        """检查三轴数量、唯一节点/名称、共用总线与定位方式，失败抛 ValueError。"""
        if len(self.axes) != 3:
            raise ValueError("必须配置三个轴")
        names = [a.name for a in self.axes]
        if any(not isinstance(name, str) or not name.strip() for name in names) or len(set(names)) != 3:
            raise ValueError("轴名称必须非空且互不相同")
        if len({a.settings.node_id for a in self.axes}) != 3:
            raise ValueError("三个轴的节点 ID 必须互不相同")
        configured_origins = [axis.origin_position_inc is not None for axis in self.axes]
        if any(configured_origins) and not all(configured_origins):
            raise ValueError("往返固定起点 origin_position_inc 必须为三个轴同时配置或同时省略")
        for axis in self.axes:
            if axis.origin_position_inc is not None:
                integer(axis.origin_position_inc, axis.settings.min_position_inc,
                        axis.settings.max_position_inc, f"轴 {axis.name} 固定往返起点")
        buses = {(a.settings.interface, a.settings.channel, a.settings.bitrate) for a in self.axes}
        if len(buses) != 1:
            raise ValueError("三轴必须使用同一 CAN 接口、通道和波特率")
        if len({a.motion.relative for a in self.axes}) != 1:
            raise ValueError("共享控制字要求三轴统一采用相对定位或绝对定位")
        integer(self.rpdo_number, 1, 8, "RPDO 编号")
        integer(self.shared_cob_id, 0x200, 0x57F, "共享 PDO COB-ID")
        if type(self.pdo_configuration_verified) is not bool:
            raise ValueError("pdo_configuration_verified 必须是布尔值")
        if self.duration_s is not None:
            positive(self.duration_s, "共同运动时长")

    @classmethod
    def load(cls, filename) -> "GroupSettings":
        """从 JSON 读取三轴配置；默认值与各轴覆盖项合并，EDS 相对配置文件解析。

        不识别的设置或无效 JSON 抛 TypeError/ValueError，文件错误抛 OSError。
        """
        path = Path(filename).resolve()
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
        if not isinstance(data, dict) or not isinstance(data.get("axes"), list):
            raise ValueError("配置必须包含 axes 数组")
        defaults = data.pop("defaults", {})
        if not isinstance(defaults, dict):
            raise ValueError("defaults 必须是对象")
        axes = []
        for row in data.pop("axes"):
            if not isinstance(row, dict):
                raise ValueError("轴配置必须是对象")
            row = dict(row)
            name = row.pop("name")
            motion = Motion(**row.pop("motion"))
            origin = row.pop("origin_position_inc", None)
            settings = Settings(**dict(defaults, **row))
            settings = replace(settings, eds=str((path.parent / settings.eds).resolve()))
            axes.append(AxisConfig(name, settings, motion, origin))
        return cls(axes=tuple(axes), **data)


@dataclass(frozen=True)
class AxisPlan:
    """缩放后的单轴轮廓、预期绝对终点及量化后的估算时长（秒）。"""

    axis: AxisConfig
    motion: Motion
    end_inc: int
    estimated_duration_s: float


@dataclass(frozen=True)
class GroupPlan:
    """三轴共同目标时间与量化轮廓；时间差为模型估算，不是实测同步精度。"""

    axes: Tuple[AxisPlan, ...]
    duration_s: float
    estimated_arrival_spread_s: float


def profile_time(distance_rev: float, velocity_rps: float, acceleration: float,
                 deceleration: float) -> Tuple[float, float]:
    """返回静止起终点梯形/三角轮廓的 (时长秒, 峰值 rps)，距离为非负电机圈数。"""
    if distance_rev == 0:
        return 0.0, 0.0
    peak = min(velocity_rps, math.sqrt(2 * distance_rev / (1 / acceleration + 1 / deceleration)))
    ramps = 0.5 * peak * peak * (1 / acceleration + 1 / deceleration)
    return peak / acceleration + peak / deceleration + max(0, distance_rev - ramps) / peak, peak


def plan_group(settings: GroupSettings, starts: Tuple[int, ...]) -> GroupPlan:
    """将三轴轮廓等比例拉长至共同时间；参数或设备量化无法实现时抛 ValueError。

    对短时轴采用 v'=v*s、a'=a*s²、d'=d*s²（s=该轴最短时间/共同时间），
    以此为初值，按设备整数单位选择不超过原请求上限的加减速度，并重新
    求解达到共同时间所需的巡航速度，减小量化引起的到达时差。
    零位移轴不参与运动时间比较。
    """
    if len(starts) != 3:
        raise ValueError("必须提供三轴起始位置")
    profiles = []
    for axis, start in zip(settings.axes, starts):
        end = axis.motion.validate(axis.settings, start)
        distance = abs(end - start) / axis.settings.encoder_counts_per_rev
        counts = axis.settings.encoder_counts_per_rev
        velocity_unit = 1875 / (512 * counts) / 60
        acceleration_unit = 4000000 / (65536 * counts)
        velocity = axis.settings.velocity_raw(axis.motion.velocity_rpm) * velocity_unit
        acceleration_raw = axis.settings.acceleration_raw(axis.motion.acceleration_rps2)
        deceleration_raw = axis.settings.acceleration_raw(axis.motion.deceleration_rps2)
        duration, peak = profile_time(distance, velocity, acceleration_raw * acceleration_unit,
                                      deceleration_raw * acceleration_unit)
        profiles.append((axis, start, end, distance, duration, peak, acceleration_raw, deceleration_raw))
    minimum = max(p[4] for p in profiles)
    common = settings.duration_s if settings.duration_s is not None else minimum
    if common + 1e-9 < minimum:
        raise ValueError(f"共同时间 {common:.6f}s 短于最短可行时间 {minimum:.6f}s")
    plans = []
    moving_times = []
    for axis, start, end, distance, duration, peak, acceleration_cap, deceleration_cap in profiles:
        motion = axis.motion
        estimated = 0.0
        if distance:
            scale = min(1.0, duration / common)
            counts = axis.settings.encoder_counts_per_rev
            unit = 4000000 / (65536 * counts)
            acceleration_raw = min(acceleration_cap, max(1, math.ceil(acceleration_cap * scale * scale)))
            deceleration_raw = min(deceleration_cap, max(1, math.ceil(deceleration_cap * scale * scale)))
            acceleration = acceleration_raw * unit
            deceleration = deceleration_raw * unit
            # T = distance/v + k*v。选择较小根，避免极短轴的浮点消减误差。
            k = 0.5 * (1 / acceleration + 1 / deceleration)
            discriminant = max(0, common * common - 4 * k * distance)
            velocity = 2 * distance / (common + math.sqrt(discriminant))
            # 留 1/4 LSB 的浮点余量，确保再次编码仍得到选中的整数；
            # 若已触及原上限，则直接使用原输入，绝不增加实际加减速度上限。
            acceleration_input = (motion.acceleration_rps2 if acceleration_raw == acceleration_cap
                                  else (acceleration_raw + 0.25) * unit)
            deceleration_input = (motion.deceleration_rps2 if deceleration_raw == deceleration_cap
                                  else (deceleration_raw + 0.25) * unit)
            motion = replace(motion, velocity_rpm=min(motion.velocity_rpm, velocity * 60),
                             acceleration_rps2=acceleration_input,
                             deceleration_rps2=deceleration_input)
            motion.validate(axis.settings, start)
            velocity = axis.settings.velocity_raw(motion.velocity_rpm) * 1875 / (512 * counts) / 60
            acceleration = axis.settings.acceleration_raw(motion.acceleration_rps2) * 4000000 / (65536 * counts)
            deceleration = axis.settings.acceleration_raw(motion.deceleration_rps2) * 4000000 / (65536 * counts)
            estimated, _ = profile_time(distance, velocity, acceleration, deceleration)
            moving_times.append(estimated)
        plans.append(AxisPlan(axis, motion, end, estimated))
    if moving_times and max(moving_times) >= min(a.settings.motion_timeout_s for a in settings.axes):
        raise ValueError("共同轮廓估算时间超过轴的 motion_timeout_s，请调整时间配置")
    spread = max(moving_times) - min(moving_times) if moving_times else 0.0
    return GroupPlan(tuple(plans), common, spread)
