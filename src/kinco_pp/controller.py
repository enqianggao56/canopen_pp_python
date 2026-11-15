"""PP 控制流程；依据手册 §5.1、§5.4、§5.7.3、§7.3、表 9-11。

控制器使用同步 SDO，适合单轴定位调试，不提供实时多轴插补。
同一实例和同一节点不得被多个线程或上位机同时控制。
"""

import logging
import time
from typing import Callable

from .model import Motion, Settings
from .objects import (
    CW_CHANGE_IMMEDIATELY, CW_ENABLE, CW_HALT, CW_NEW_SETPOINT,
    CW_RELATIVE, CW_SHUTDOWN, CW_SWITCH_ON, Object, PP_MODE,
    STATE_ENABLED, STATE_MASK, STATE_READY, STATE_SWITCHED_ON,
    SW_FAULT, SW_FOLLOWING_ERROR, SW_LIMIT_ACTIVE, SW_SETPOINT_ACK,
    SW_TARGET_REACHED,
)
from .transport import DriveIO

LOG = logging.getLogger(__name__)


class DriveError(RuntimeError):
    """故障、限位、跟随误差或非预期驱动器状态。"""


class DriveTimeout(DriveError):
    """驱动器状态或运动未在配置时间内完成。"""


class ProfilePositionController:
    """执行单次 PP 运动；出现异常后调用方必须尝试 halt 并关闭通信。"""

    def __init__(self, io: DriveIO, settings: Settings,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep):
        """保存通信接口及配置；clock/sleep 用于测试注入虚拟时间，无返回值。"""
        self.io = io
        self.settings = settings
        self.clock = clock
        self.sleep = sleep
        self.controlword = CW_SHUTDOWN
        self.enable_started = False
        self.prepared = False

    def _control(self, value: int) -> None:
        """记录并发送控制字；即使响应丢失也保留最后尝试值，便于暂停。"""
        self.controlword = value
        self.io.write(Object.CONTROLWORD, value)

    def _status(self) -> int:
        """读取状态字；故障、限位或位置跟随误差抛 DriveError，含原始状态字。"""
        status = self.io.read(Object.STATUSWORD)
        if status & (SW_FAULT | SW_FOLLOWING_ERROR | SW_LIMIT_ACTIVE):
            raise DriveError(f"驱动器故障、限位或跟随误差，状态字=0x{status:04X}")
        return status

    def _wait(self, predicate: Callable[[int], bool], timeout: float, label: str) -> int:
        """轮询状态并返回满足 predicate 的状态字；超时抛 DriveTimeout。

        timeout 单位秒；每次 SDO 另受 sdo_timeout_s 限制，等待并非硬实时。
        """
        deadline = self.clock() + timeout
        while True:
            status = self._status()
            if predicate(status):
                return status
            if self.clock() >= deadline:
                raise DriveTimeout(f"等待{label}超时，状态字=0x{status:04X}")
            self.sleep(min(self.settings.poll_interval_s, max(0, deadline - self.clock())))

    def prepare(self) -> None:
        """进入 PP 并使能，保持 Halt，不触发位置指令；状态异常抛 DriveError。

        §5.1：shutdown → switched on → operation enabled。
        §5.7.3：设置 0x605D=1，使暂停使用 0x6084 减速度，结束后保持使能。
        不自动清除故障，不修改持久参数或其他节点。
        """
        initial = self._status()
        if initial & STATE_MASK == STATE_ENABLED:
            # 上一次运行结束后仍锁轴时直接复用，避免先 shutdown 造成松轴。
            if (self.io.read(Object.MODE_DISPLAY) != PP_MODE
                    or not self.io.read(Object.CONTROLWORD) & CW_HALT
                    or not self._speed_is_zero()):
                raise DriveError("驱动器已使能但未处于静止的 PP Halt 状态，拒绝接管")
            self._wait_standstill(self.settings.state_timeout_s)
            self.enable_started = True
            self._control(CW_ENABLE | CW_HALT)
            self.io.write(Object.HALT_OPTION, 1)
            self.prepared = True
            return
        self.enable_started = True
        self._control(CW_SHUTDOWN | CW_HALT)
        self._wait(lambda status: status & STATE_MASK == STATE_READY,
                   self.settings.state_timeout_s, "运行就绪")
        self.io.write(Object.HALT_OPTION, 1)
        self.io.write(Object.MODE, PP_MODE)
        # 部分固件在使能后才更新 0x6061：先核对 0x6060 回读，
        # 保持 Halt 进入使能，再确认实际模式，确认前绝不触发新位置。
        self._wait(lambda status: self.io.read(Object.MODE) == PP_MODE,
                   self.settings.state_timeout_s, "PP 模式设置回读")
        self._control(CW_SWITCH_ON | CW_HALT)
        self._wait(lambda status: status & STATE_MASK == STATE_SWITCHED_ON,
                   self.settings.state_timeout_s, "主回路就绪")
        self._control(CW_ENABLE | CW_HALT)
        self._wait(lambda status: status & STATE_MASK == STATE_ENABLED,
                   self.settings.state_timeout_s, "驱动器使能")
        self._wait(lambda status: self.io.read(Object.MODE_DISPLAY) == PP_MODE,
                   self.settings.state_timeout_s, "PP 实际模式确认")
        self._wait_standstill(self.settings.state_timeout_s)
        self.prepared = True

    def _speed_is_zero(self) -> bool:
        """按手册 §7.2 换算 0x606C，以配置的 rpm 容差判断速度反馈接近零。"""
        raw = self.io.read(Object.ACTUAL_VELOCITY)
        rpm = abs(raw) * 1875 / (512 * self.settings.encoder_counts_per_rev)
        return rpm <= self.settings.zero_velocity_tolerance_rpm

    def _wait_standstill(self, timeout: float) -> None:
        """同时确认低速与持续位置稳定，避免速度噪声或单次零值误判停稳。

        timeout 单位秒；位置相对确认窗口起点变化超过到位容差时重新计时。
        故障、读数异常及超时分别通过原有异常路径报告。
        """
        anchor = None
        stable_since = None

        def stationary(status: int) -> bool:
            """更新静止采样窗口，仅在完整稳定时长后返回 True。"""
            nonlocal anchor, stable_since
            position = self.io.read(Object.ACTUAL_POSITION)
            now = self.clock()
            if not self._speed_is_zero():
                anchor = None
                stable_since = None
                return False
            if anchor is None or abs(position - anchor) > self.settings.position_tolerance_inc:
                anchor = position
                stable_since = now
                return False
            return now - stable_since >= self.settings.standstill_duration_s

        self._wait(stationary, timeout, "速度与位置稳定")

    def move(self, motion: Motion) -> int:
        """执行一次定位并返回实际到位位置 inc；参数错误、超时或故障抛异常。

        必须先 prepare。相对目标根据触发前静止位置计算；绝对定位要求已确认原点。
        先在 Halt 下清除 bit4 并等待 bit12=0；目标写入后，以 bit4 上升沿
        同时解除 Halt；等待 bit12=1 后清除 bit4，并等待 bit12=0。
        到位必须同时满足 bit10 和实际位置容差，避免误用上一指令的到位标志。
        """
        if not self.prepared:
            raise DriveError("必须先调用 prepare()")
        status = self._status()
        if status & STATE_MASK != STATE_ENABLED:
            raise DriveError(f"驱动器未使能，状态字=0x{status:04X}")
        if self.io.read(Object.MODE_DISPLAY) != PP_MODE:
            raise DriveError("驱动器已离开 PP 模式")
        start = self.io.read(Object.ACTUAL_POSITION)
        end = motion.validate(self.settings, start)
        base = CW_ENABLE | (CW_RELATIVE if motion.relative else CW_CHANGE_IMMEDIATELY)
        # §5.4 绝对 0x2F→0x3F，相对 0x4F→0x5F；准备期间额外置 Halt。
        self._control(base | CW_HALT)
        self._wait(lambda status: not status & SW_SETPOINT_ACK,
                   self.settings.state_timeout_s, "清除旧指令应答")
        self.io.write(Object.PROFILE_VELOCITY, self.settings.velocity_raw(motion.velocity_rpm))
        self.io.write(Object.PROFILE_ACCELERATION,
                      self.settings.acceleration_raw(motion.acceleration_rps2))
        self.io.write(Object.PROFILE_DECELERATION,
                      self.settings.acceleration_raw(motion.deceleration_rps2))
        self.io.write(Object.TARGET_POSITION, motion.target_inc)
        LOG.info("PP %s定位：起点=%d，终点=%d inc", "相对" if motion.relative else "绝对", start, end)
        self._control(base | CW_NEW_SETPOINT)
        self._wait(lambda status: bool(status & SW_SETPOINT_ACK),
                   self.settings.state_timeout_s, "新指令应答")
        self._control(base)
        self._wait(lambda status: not status & SW_SETPOINT_ACK,
                   self.settings.state_timeout_s, "应答复位")

        def reached(status: int) -> bool:
            """检查本次目标的位置与到位位；不依赖观察到短暂的 bit10 清零。"""
            if status & STATE_MASK != STATE_ENABLED:
                raise DriveError(f"运动期间退出使能，状态字=0x{status:04X}")
            if self.io.read(Object.MODE_DISPLAY) != PP_MODE:
                raise DriveError("运动期间退出 PP 模式")
            actual = self.io.read(Object.ACTUAL_POSITION)
            return bool(status & SW_TARGET_REACHED) and abs(actual - end) <= self.settings.position_tolerance_inc

        self._wait(reached, self.settings.motion_timeout_s, "目标到位")
        return self.io.read(Object.ACTUAL_POSITION)

    def halt(self) -> None:
        """置 bit8 并等待暂停完成，保持当前使能状态，无返回值。

        §5.1.2.1、§5.7.3：Halt 不是 STO 或硬件急停；通信失败不能保证停机。
        暂停后联合确认 0x606C 低速与位置稳定；可能抛通信、故障或超时异常。
        """
        if not self.enable_started:
            return
        self._control((self.controlword & ~CW_NEW_SETPOINT) | CW_HALT)
        self._wait_standstill(self.settings.motion_timeout_s)
        self.prepared = False
