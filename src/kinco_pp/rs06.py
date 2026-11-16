"""RobStride RS06 CANopen PP 适配；依据 RS06 手册 §5.3、§5.4、§5.6。

位置每圈 16384 inc，速度 0.1 rpm，加速度 0.1 rpm/s；不能使用 Kinco 换算。
RS00/EL05手册§5.3/§5.4/§5.6逐项一致的PP流程由本模块复用；力矩与设备校验独立配置。
PP 在控制字 15 下更新绝对位置即可运行，不使用 bit4/bit12 握手。
"""

import json
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path

from .controller import DriveError, DriveTimeout
from .group_model import profile_time
from .model import integer, positive
from .objects import (CW_ENABLE, CW_SHUTDOWN, CW_SWITCH_ON, Object, PP_MODE,
                      STATE_ENABLED, STATE_MASK, STATE_READY, STATE_SWITCHED_ON,
                      SW_FAULT, SW_TARGET_REACHED, INT32_MIN, INT32_MAX)

RS06_COUNTS_PER_REV = 16384
RS06_DISABLE = 1
RS06_QUICK_STOP = 11
# 手册§5.3状态机对应的DS402状态值，配合STATE_MASK检查。
STATE_SWITCH_ON_DISABLED = 0x40
STATE_QUICK_STOP_ACTIVE = 0x07
POSITION_WINDOW = 0x6067
POSITION_WINDOW_TIME = 0x6068
TORQUE_LIMIT = 0x6071
WATCHDOG = 0x6099
WATCHDOG_TICKS_PER_SECOND = 20000


@dataclass(frozen=True)
class RS06Settings:
    """第四轴独立配置；固定端点 inc、速度 rpm、加速时间秒、力矩为 0.1% 负载率。

    hardware_verified 包含固件/EDS、CANopen 协议、停止方式及 acc_stutus=0 的
    人工确认；后者按手册 §4.2.10 使加速度兼作减速度，CANopen 表未说明其设置地址。
    本适配不猜测 0x6084 的单位，不写入该对象，也不修改私有协议参数。
    synchronous_pdo_verified包含同步RPDO4及临时TPDO1/2槽位、COB-ID与SYNC核对。
    """

    eds: str = '../RS06.eds'
    interface: str = 'socketcan'
    channel: str = 'can0'
    bitrate: int = 1000000
    node_id: int = 4
    expected_vendor_id: int = 0x556
    expected_product_code: int = 0x4644
    origin_position_inc: int = 0
    far_position_inc: int = 1638400
    min_position_inc: int = -25
    max_position_inc: int = 1638425
    velocity_rpm: float = 100
    acceleration_time_s: float = 2
    torque_limit_raw: int = 100
    position_tolerance_inc: int = 25
    position_window_time_ms: int = 100
    zero_velocity_tolerance_rpm: float = 0
    standstill_duration_s: float = 0.1
    sdo_timeout_s: float = 0.3
    sdo_pause_s: float = 0.005
    state_timeout_s: float = 3
    motion_timeout_s: float = 90
    poll_interval_s: float = 0.02
    watchdog_timeout_s: float = 1
    target_cob_id: int = 0x504
    sync_settle_s: float = 0.003
    hardware_verified: bool = False
    absolute_reference_verified: bool = False
    synchronous_pdo_verified: bool = False

    def __post_init__(self):
        """拒绝无效端点、速度、时限及未说明的单位；错误抛 ValueError。"""
        integer(self.node_id, 1, 127, 'RS06 节点 ID')
        integer(self.bitrate, 1, 1000000, 'RS06 波特率')
        integer(self.expected_vendor_id, 0, (1 << 32) - 1, 'RS06 Vendor ID')
        integer(self.expected_product_code, 0, (1 << 32) - 1, 'RS06 Product Code')
        integer(self.min_position_inc, INT32_MIN, INT32_MAX, 'RS06 最小位置')
        integer(self.max_position_inc, INT32_MIN, INT32_MAX, 'RS06 最大位置')
        if self.min_position_inc >= self.max_position_inc:
            raise ValueError('RS06 软件范围必须递增')
        for name in ('origin_position_inc', 'far_position_inc'):
            integer(getattr(self, name), self.min_position_inc, self.max_position_inc, name)
        if self.far_position_inc <= self.origin_position_inc:
            raise ValueError('RS06 远端必须大于固定起点')
        integer(self.torque_limit_raw, 1, 1000, 'RS06 力矩上限')
        integer(self.position_tolerance_inc, 0, INT32_MAX, 'RS06 位置容差')
        if self.far_position_inc - self.origin_position_inc <= 2 * self.position_tolerance_inc:
            raise ValueError('RS06 行程必须大于两倍到位容差')
        integer(self.position_window_time_ms, 1, 65535, 'RS06 到位窗口毫秒')
        integer(self.target_cob_id, 0x200, 0x57F, 'RS06 目标 RPDO COB-ID')
        for name in ('velocity_rpm', 'acceleration_time_s', 'standstill_duration_s',
                     'sdo_timeout_s', 'state_timeout_s', 'motion_timeout_s',
                     'poll_interval_s', 'watchdog_timeout_s', 'sync_settle_s', 'sdo_pause_s'):
            positive(getattr(self, name), name)
        if self.velocity_rpm > 100:
            raise ValueError('本次 RS06 接入速度上限为 100 RPM')
        if self.standstill_duration_s >= min(self.state_timeout_s, self.motion_timeout_s):
            raise ValueError('RS06 静止确认时间必须小于状态和运动超时')
        if (isinstance(self.zero_velocity_tolerance_rpm, bool)
                or not isinstance(self.zero_velocity_tolerance_rpm, (int, float))
                or not math.isfinite(self.zero_velocity_tolerance_rpm)
                or self.zero_velocity_tolerance_rpm < 0):
            raise ValueError('RS06 零速容差必须有限且非负')
        integer(round(self.watchdog_timeout_s * WATCHDOG_TICKS_PER_SECOND),
                1, INT32_MAX, 'RS06 看门狗计数')
        for name in ('hardware_verified', 'absolute_reference_verified', 'synchronous_pdo_verified'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} 必须是布尔值')
        for name in ('eds', 'interface', 'channel'):
            if not isinstance(getattr(self, name), str) or not getattr(self, name):
                raise ValueError(f'{name} 必须是非空字符串')
        self.velocity_raw(self.velocity_rpm)
        self.acceleration_raw(self.velocity_rpm / 60 / self.acceleration_time_s)

    @classmethod
    def load(cls, filename):
        """读取独立 RS06 JSON，EDS 相对于配置文件解析；文件/字段错误直接抛出。"""
        path = Path(filename).resolve()
        with path.open(encoding='utf-8') as stream:
            settings = cls(**json.load(stream))
        return replace(settings, eds=str((path.parent / settings.eds).resolve()))

    @staticmethod
    def velocity_raw(rpm):
        """按 §5.6 编码 0x6081，单位 0.1 rpm，向下取整；量化为零抛 ValueError。"""
        positive(rpm, 'RS06 速度')
        raw = int(rpm * 10)
        integer(raw, 1, (1 << 32) - 1, 'RS06 原始速度')
        return raw

    @staticmethod
    def acceleration_raw(rps2):
        """按 §5.6 编码 0x6083，rps/s × 60 × 10，向下取整。"""
        positive(rps2, 'RS06 加速度')
        raw = int(rps2 * 600)
        integer(raw, 1, (1 << 32) - 1, 'RS06 原始加速度')
        return raw

    @staticmethod
    def feedback_velocity_rpm(raw):
        """按 §5.4 将 0x606C 有符号原始值解码为 rpm。"""
        return raw / 10


class RS06Peer:
    """与 Kinco 三轴协同的第四轴：独立 PP 轮廓及到位监视，共同 SYNC 启动。"""

    name = 'rs06/节点4'

    def __init__(self, io, settings, clock=time.monotonic, sleep=time.sleep,
                 enter_preoperational=None):
        """注入通信、配置、时钟及预操作态回调；运动期反馈由TPDO缓存提供。

        准备/停止仍用SDO；实机同步PDO下状态控制也受SYNC门控。
        """
        self.io, self.settings, self.clock, self.sleep = io, settings, clock, sleep
        self.enter_preoperational = enter_preoperational
        self.name = f"{getattr(settings, 'model', 'RS06').lower()}/节点{settings.node_id}"
        self.enable_started = False
        self.prepared = False
        self.triggered_at = None
        self.arrived_at = None
        self.complete = False
        self.arrival_counted = False
        self.target = settings.far_position_inc
        self.velocity_raw = settings.velocity_raw(settings.velocity_rpm)
        self.acceleration_raw = settings.acceleration_raw(
            settings.velocity_rpm / 60 / settings.acceleration_time_s)
        self.estimated_duration_s = 0.0
        self.stable_since = None
        self.completed_legs = 0
        self.completed_full_legs = 0

    def check(self):
        """只读检查固定区间与实际位置，返回端点；不重设坐标或放宽范围。"""
        position = self.io.read(Object.ACTUAL_POSITION)
        self._check_position(position)
        low, high = self.settings.origin_position_inc, self.settings.far_position_inc
        tolerance = self.settings.position_tolerance_inc
        if not low - tolerance <= position <= high + tolerance:
            raise ValueError(f'{self.name} 当前 {position} inc 不在已确认往返区间 [{low}, {high}]')
        return dict(node_id=self.settings.node_id, start_position_inc=position,
                    origin_position_inc=low, far_position_inc=high,
                    turns=(high - low) / RS06_COUNTS_PER_REV)

    def _check_position(self, position):
        """实际位置超出软件范围立即抛 DriveError，包含轴名和原始位置。"""
        if not self.settings.min_position_inc <= position <= self.settings.max_position_inc:
            raise DriveError(f'{self.name} 位置 {position} inc 超出软件范围')

    def plan(self, target, common_duration_s=None, start_position_inc=None):
        """为固定端点规划整数轮廓，返回估算秒数；上限为 100 RPM 和配置加速度。

        对短恢复段按共同时间缩放轮廓，在相邻整数速度中选模型时间最接近者。
        量化可能产生模型时差，由四轴报告单独输出，不能当作实测精度。
        """
        if target not in (self.settings.origin_position_inc, self.settings.far_position_inc):
            raise ValueError(f'{self.name} 目标必须为已确认固定端点')
        current = self.check()['start_position_inc'] if start_position_inc is None else start_position_inc
        self._check_position(current)
        self.target = target
        distance = abs(target - current) / RS06_COUNTS_PER_REV
        velocity_cap = self.settings.velocity_raw(self.settings.velocity_rpm)
        acceleration_cap = self.settings.acceleration_raw(
            self.settings.velocity_rpm / 60 / self.settings.acceleration_time_s)
        minimum, _ = profile_time(distance, velocity_cap / 600,
                                  acceleration_cap / 600, acceleration_cap / 600)
        common = minimum if common_duration_s is None else common_duration_s
        if common + 1e-9 < minimum:
            raise ValueError(f'共同时间短于 {self.name} 最快可行轮廓')
        if distance:
            scale = min(1, minimum / common)
            acceleration = min(acceleration_cap, max(1, math.ceil(acceleration_cap * scale * scale)))
            k = 600 / acceleration
            v = 2 * distance / (common + math.sqrt(max(0, common * common - 4 * k * distance)))
            candidates = {max(1, min(velocity_cap, math.floor(v * 600))),
                          max(1, min(velocity_cap, math.ceil(v * 600)))}
            velocity = min(candidates, key=lambda raw: abs(profile_time(
                distance, raw / 600, acceleration / 600, acceleration / 600)[0] - common))
            duration, _ = profile_time(distance, velocity / 600, acceleration / 600, acceleration / 600)
            self.velocity_raw, self.acceleration_raw = velocity, acceleration
        else:
            duration = 0.0
        if duration >= self.settings.motion_timeout_s:
            raise ValueError(f'{self.name} 轮廓估算时间超过 motion_timeout_s')
        self.estimated_duration_s = duration
        return duration

    def preflight(self):
        """在任何轴使能前检查区间、故障及接管条件；失败不写控制对象。"""
        self.check()
        status = self.io.read(Object.STATUSWORD)
        if status & SW_FAULT:
            raise DriveError(f'{self.name} 故障，状态字=0x{status:04X}')
        if status & STATE_MASK == STATE_ENABLED:
            raise DriveError(f'{self.name} 已由其他控制流程使能，拒绝接管')
        if abs(self.settings.feedback_velocity_rpm(self.io.read(Object.ACTUAL_VELOCITY))) > self.settings.zero_velocity_tolerance_rpm:
            raise DriveError(f'{self.name} 尚未静止，拒绝接管')

    def _wait(self, predicate, timeout, label):
        """按秒轮询条件，SDO 本身另有通信超时；失败抛 DriveTimeout。"""
        deadline = self.clock() + timeout
        while not predicate():
            if self.clock() >= deadline:
                raise DriveTimeout(f'{self.name} 等待{label}超时')
            self.sleep(self.settings.poll_interval_s)

    def _stationary(self):
        """返回速度及位置持续稳定的检查函数；不因故障位跳过实际停稳确认。"""
        anchor, since = None, None
        def sample():
            """速度或位置超出容差时重启稳定窗口。"""
            nonlocal anchor, since
            position = self.io.read(Object.ACTUAL_POSITION)
            velocity = abs(self.settings.feedback_velocity_rpm(self.io.read(Object.ACTUAL_VELOCITY)))
            if velocity > self.settings.zero_velocity_tolerance_rpm:
                anchor, since = None, None
                return False
            if anchor is None or abs(position - anchor) > self.settings.position_tolerance_inc:
                anchor, since = position, self.clock()
                return False
            return self.clock() - since >= self.settings.standstill_duration_s
        return sample

    def prepare(self):
        """按 §5.3/§5.6 失能配置 PP、力矩、速度、加速度和位置窗口，再使能保持。

        先将目标设为当前绝对位置，避免重新使能执行旧目标；不使用模式6标零。
        0x6099:01 看门狗按 20000 tick/s 写入且回读，断线时由电机失能。
        """
        self._wait(self._stationary(), self.settings.state_timeout_s, '准备前停稳')
        if self.enter_preoperational is not None:
            self.enter_preoperational()
        self.enable_started = True
        self.io.write(Object.CONTROLWORD, RS06_DISABLE)
        self._wait(lambda: self.io.read(Object.STATUSWORD) & STATE_MASK == STATE_SWITCH_ON_DISABLED,
                   self.settings.state_timeout_s, '失能')
        current = self.io.read(Object.ACTUAL_POSITION)
        self._check_position(current)
        opposite = (self.settings.origin_position_inc if self.target == self.settings.far_position_inc
                    else self.settings.far_position_inc)
        self.full_stroke = abs(current - opposite) <= self.settings.position_tolerance_inc
        self.has_motion = abs(current - self.target) > self.settings.position_tolerance_inc
        parameters = [(Object.MODE, PP_MODE, 0), (TORQUE_LIMIT, self.settings.torque_limit_raw, 0),
                      (Object.PROFILE_VELOCITY, self.velocity_raw, 0),
                      (Object.PROFILE_ACCELERATION, self.acceleration_raw, 0),
                      (POSITION_WINDOW, self.settings.position_tolerance_inc, 0),
                      (POSITION_WINDOW_TIME, self.settings.position_window_time_ms, 0),
                      (WATCHDOG, round(self.settings.watchdog_timeout_s * WATCHDOG_TICKS_PER_SECOND), 1),
                      (Object.TARGET_POSITION, current, 0)]
        for index, value, subindex in parameters:
            self.io.write(index, value, subindex)
            observed = self.io.read(index, subindex)
            if index == Object.TARGET_POSITION:
                # 实机失能时0x607A会保持在当前反馈附近，当前坐标本身也有1~2inc抖动。
                # 此写入仅用于防止使能执行旧远端；检查其仍位于当前位置窗口，
                # 其余模式、速度、加速度和力矩对象仍要求完全相等。
                self._check_position(observed)
                matches = abs(observed - current) <= self.settings.position_tolerance_inc
            else:
                matches = observed == value
            if not matches:
                raise DriveError(f'{self.name} 0x{index:04X}:{subindex:02X} 参数回读不一致：'
                                 f'写入={value}，回读={observed}')
        # 按§5.3使用完整的6→7→15状态序列，逐步确认实际状态。
        # 同步PDO实机须在NMT预操作态准备，否则SDO控制字也会等待SYNC。
        for controlword, expected, label in ((CW_SHUTDOWN, STATE_READY, '运行就绪'),
                                            (CW_SWITCH_ON, STATE_SWITCHED_ON, '主回路就绪')):
            self.io.write(Object.CONTROLWORD, controlword)
            self._wait(lambda: self.io.read(Object.STATUSWORD) & STATE_MASK == expected,
                       self.settings.state_timeout_s, label)
        self.io.write(Object.CONTROLWORD, CW_ENABLE)
        def enabled():
            """实际模式和状态同时确认，故障立即退出。"""
            status = self.io.read(Object.STATUSWORD)
            if status & SW_FAULT:
                raise DriveError(f'{self.name} 使能故障，状态字=0x{status:04X}')
            return (status & STATE_MASK == STATE_ENABLED
                    and self.io.read(Object.MODE_DISPLAY) == PP_MODE)
        self._wait(enabled, self.settings.state_timeout_s, 'PP 使能')
        self._wait(self._stationary(), self.settings.state_timeout_s, '使能后保持')
        self.complete, self.prepared = False, True
        self.arrival_counted = False
        self.triggered_at, self.arrived_at, self.stable_since = None, None, None

    def trigger(self):
        """总线已用共同 SYNC 触发后记录主机时间；不重复写目标、不等待 bit12。"""
        if not self.prepared:
            raise DriveError(f'{self.name} 未准备完成')
        self.triggered_at = self.clock()

    def poll(self):
        """检查模式、故障、实际范围、超时及持续到位；异常交由全组停止路径处理。"""
        if not self.prepared:
            return
        status = self.io.read(Object.STATUSWORD)
        if status & SW_FAULT or status & STATE_MASK != STATE_ENABLED:
            raise DriveError(f'{self.name} 故障或退出使能，状态字=0x{status:04X}')
        if self.io.read(Object.MODE_DISPLAY) != PP_MODE:
            raise DriveError(f'{self.name} 已退出 PP 模式')
        position = self.io.read(Object.ACTUAL_POSITION)
        self._check_position(position)
        if self.triggered_at is None:
            return
        velocity = abs(self.settings.feedback_velocity_rpm(self.io.read(Object.ACTUAL_VELOCITY)))
        reached = (bool(status & SW_TARGET_REACHED)
                   and abs(position - self.target) <= self.settings.position_tolerance_inc
                   and velocity <= self.settings.zero_velocity_tolerance_rpm)
        if not reached:
            if self.arrival_counted and abs(position - self.target) > self.settings.position_tolerance_inc:
                raise DriveError(f'{self.name} 到位后离开目标窗口')
            # 位置仍在窗口内的速度反馈尖峰只重新确认稳定窗口，不能冒充位置跑离。
            self.complete = False
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = self.clock()
        elif not self.complete and self.clock() - self.stable_since >= max(
                self.settings.standstill_duration_s, self.settings.position_window_time_ms / 1000):
            self.complete, self.arrived_at = True, self.clock()
            if not self.arrival_counted:
                self.arrival_counted = True
                self.completed_legs += int(self.has_motion)
                self.completed_full_legs += int(self.full_stroke)
        if not self.complete and self.clock() - self.triggered_at >= self.settings.motion_timeout_s:
            raise DriveTimeout(f'{self.name} 等待目标 {self.target} inc 到位超时')

    def request_halt(self):
        """按 §5.3 发送控制字11急停；异常仍保留使能尝试标志，供后续确认。"""
        if self.enable_started:
            try:
                if self.enter_preoperational is not None:
                    # 先关闭同步门控，再发停止字，避免停止也等待SYNC。
                    # 不发额外SYNC，以免释放尚未确认的残留运动目标。
                    self.enter_preoperational()
            finally:
                # 关闭门控的通信失败也不能跳过最后的厂家停止字请求。
                self.io.write(Object.CONTROLWORD, RS06_QUICK_STOP)

    def wait_halted(self):
        """确认持续低速且位置稳定；不发送失能以冒充停稳，不清除故障。"""
        if self.enable_started:
            self._wait(self._stationary(), self.settings.motion_timeout_s, '停止后速度与位置稳定')
            self.prepared = False


class SimulatedRS06:
    """RS06 独立协议模拟器；直接绝对目标、无 bit12 应答，不模拟机械时长。"""

    def __init__(self, position=0):
        """初始化失能轴与写入记录；不连接任何 CAN 接口。"""
        self.values = {Object.STATUSWORD: STATE_SWITCH_ON_DISABLED, Object.CONTROLWORD: RS06_DISABLE,
                       Object.ACTUAL_POSITION: position, Object.ACTUAL_VELOCITY: 0,
                       Object.MODE: 0, Object.MODE_DISPLAY: 0, Object.TARGET_POSITION: position}
        self.writes, self.remaining_polls = [], 0

    def read(self, index, subindex=0):
        """读取对象；状态轮询推进模拟目标，第一轮保留旧到位位以验证过滤。"""
        if index == Object.STATUSWORD and self.remaining_polls:
            self.remaining_polls -= 1
            if self.remaining_polls == 0:
                self.values[Object.ACTUAL_POSITION] = self.values[Object.TARGET_POSITION]
                self.values[Object.ACTUAL_VELOCITY] = 0
                self.values[Object.STATUSWORD] |= SW_TARGET_REACHED
        return self.values[(index, subindex) if subindex else index]

    def write(self, index, value, subindex=0):
        """处理 PP 使能、绝对目标和厂家停止字；意外 bit4/bit8 控制字立即报错。"""
        self.writes.append((index, value, subindex))
        self.values[(index, subindex) if subindex else index] = value
        if index == Object.MODE:
            self.values[Object.MODE_DISPLAY] = value
        elif index == Object.CONTROLWORD:
            if value == CW_SHUTDOWN:
                self.values[Object.STATUSWORD] = STATE_READY
            elif value == CW_SWITCH_ON:
                self.values[Object.STATUSWORD] = STATE_SWITCHED_ON
            elif value == CW_ENABLE:
                self.values[Object.STATUSWORD] = STATE_ENABLED | SW_TARGET_REACHED
            elif value in (RS06_DISABLE, RS06_QUICK_STOP):
                self.values[Object.STATUSWORD] = (STATE_SWITCH_ON_DISABLED if value == RS06_DISABLE
                                                 else STATE_QUICK_STOP_ACTIVE)
                self.values[Object.ACTUAL_VELOCITY] = 0
                self.remaining_polls = 0
            else:
                raise ValueError('RS06 不应收到 Kinco 触发/暂停控制字')
        elif index == Object.TARGET_POSITION and self.values[Object.CONTROLWORD] == CW_ENABLE:
            self.remaining_polls = 6
            self.values[Object.ACTUAL_VELOCITY] = self.values.get(Object.PROFILE_VELOCITY, 1000)
