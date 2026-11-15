"""三轴 PP 编排：预装载、单帧共同触发、逐轴监视、全组暂停与配置恢复。"""

import logging
import time
from dataclasses import dataclass
from typing import Tuple

from .controller import DriveError, DriveTimeout, ProfilePositionController
from .group_model import GroupPlan, GroupSettings, plan_group
from .objects import (
    CW_ENABLE, CW_HALT, CW_NEW_SETPOINT, Object, PP_MODE,
    STATE_ENABLED, STATE_MASK, SW_FAULT, SW_FOLLOWING_ERROR, SW_LIMIT_ACTIVE,
    SW_SETPOINT_ACK, SW_TARGET_REACHED,
)

LOG = logging.getLogger(__name__)


class GroupError(DriveError):
    """三轴运行或停止/恢复失败，消息保留各轴名称和原始错误。"""


@dataclass(frozen=True)
class GroupResult:
    """到位位置 inc、规划结果和轮询观察到的到达时差（秒，含轮询延迟）。"""

    positions: Tuple[int, ...]
    plan: GroupPlan
    observed_arrival_spread_s: float


class ThreeAxisController:
    """控制同一总线的三个轴；bus 提供三个 IO、广播、配置及恢复方法。"""

    def __init__(self, bus, settings: GroupSettings, clock=time.monotonic, sleep=time.sleep):
        """注入总线、配置及可测试的时间函数；不发送任何控制指令。"""
        if len(bus.axes) != 3:
            raise ValueError("总线必须提供三个轴")
        self.bus = bus
        self.settings = settings
        self.clock = clock
        self.sleep = sleep
        self.controllers = [ProfilePositionController(io, axis.settings, clock, sleep)
                            for io, axis in zip(bus.axes, settings.axes)]
        self._broadcast_armed = False
        self._last_controlword = CW_ENABLE

    def _preflight(self) -> Tuple[int, ...]:
        """检查所有轴的故障、接管条件和初始目标，不改变设备状态；返回起点。"""
        starts = []
        for io, axis in zip(self.bus.axes, self.settings.axes):
            status = io.read(Object.STATUSWORD)
            if status & (SW_FAULT | SW_LIMIT_ACTIVE | SW_FOLLOWING_ERROR):
                raise GroupError(f"轴 {axis.name} 预检查失败，状态字=0x{status:04X}")
            if status & STATE_MASK == STATE_ENABLED:
                if io.read(Object.MODE_DISPLAY) != PP_MODE or not io.read(Object.CONTROLWORD) & CW_HALT:
                    raise GroupError(f"轴 {axis.name} 已使能且未处于 PP Halt，拒绝接管")
            starts.append(io.read(Object.ACTUAL_POSITION))
        plan_group(self.settings, tuple(starts))
        return tuple(starts)

    def _broadcast(self, value: int) -> None:
        """发送前记录全部轴的最后尝试值，响应不明确时仍能正确暂停。"""
        self._last_controlword = value
        for controller in self.controllers:
            controller.controlword = value
        self.bus.broadcast(value)

    def _wait_all(self, predicate, timeouts, label: str) -> None:
        """轮询每个轴并检查 PP 使能/故障，任一轴超时立即退出，不顺序阻塞等待。"""
        deadlines = [self.clock() + timeout for timeout in timeouts]
        while True:
            complete = True
            for index, (axis, controller, deadline) in enumerate(zip(self.settings.axes, self.controllers, deadlines)):
                try:
                    status = controller.check_enabled()
                    done = predicate(index, status)
                    if not done and self.clock() >= deadline:
                        raise DriveTimeout(f"等待{label}超时，状态字=0x{status:04X}")
                except Exception as exc:
                    raise GroupError(f"轴 {axis.name}: {exc}") from exc
                complete = complete and done
            if complete:
                return
            self.sleep(min(a.settings.poll_interval_s for a in self.settings.axes))

    def halt_all(self) -> None:
        """先向全组广播 Halt，再向所有已开始使能的轴发送 SDO Halt，最后确认停稳。

        一轴发送或确认失败也继续处理其余轴；汇总错误抛 GroupError。准备未完成时
        不广播 Enable，以免尚未配置 PP 的轴被意外使能。通信停止不是硬件急停。
        """
        errors = []
        if self._broadcast_armed:
            try:
                self._broadcast((self._last_controlword & ~CW_NEW_SETPOINT) | CW_HALT)
            except Exception as exc:
                errors.append(f"广播暂停: {exc}")
        for axis, controller in zip(self.settings.axes, self.controllers):
            try:
                controller.request_halt()
            except Exception as exc:
                errors.append(f"轴 {axis.name} 暂停请求: {exc}")
        for axis, controller in zip(self.settings.axes, self.controllers):
            try:
                controller.wait_halted()
            except Exception as exc:
                errors.append(f"轴 {axis.name} 停稳确认: {exc}")
        self._broadcast_armed = False
        if errors:
            raise GroupError("; ".join(errors))

    def run(self) -> GroupResult:
        """执行三轴协调定位并返回结果；退出时暂停，确认停稳后恢复临时 PDO 配置。

        不改变各轴编码器零点或机械传动比。共同控制字要求统一绝对/相对方式。
        所有目标装载并核对后只广播一次 bit4 上升沿，之后只复位应答或暂停。
        """
        primary = None
        try:
            starts = self._preflight()
            plan = plan_group(self.settings, starts)
            if all(p.end_inc == start for p, start in zip(plan.axes, starts)):
                return GroupResult(starts, plan, 0.0)
            self.bus.configure()
            self.bus.start_all()
            for axis, controller in zip(self.settings.axes, self.controllers):
                try:
                    controller.prepare()
                except Exception as exc:
                    raise GroupError(f"轴 {axis.name} 准备失败: {exc}") from exc
            # 使能过程中位置可能有微小变化，触发前重新规划及检查所有起终点。
            starts = tuple(io.read(Object.ACTUAL_POSITION) for io in self.bus.axes)
            plan = plan_group(self.settings, starts)
            stages = []
            for controller, axis_plan in zip(self.controllers, plan.axes):
                try:
                    stages.append(controller.stage(axis_plan.motion))
                except Exception as exc:
                    raise GroupError(f"轴 {axis_plan.axis.name} 装载目标失败: {exc}") from exc
            if any(base != stages[0][1] for _, base in stages):
                raise GroupError("三轴控制字不一致，未触发运动")
            # stage 内保持 Halt；在整组触发前再次验证全部应答与状态。
            state_timeouts = [axis.settings.state_timeout_s for axis in self.settings.axes]
            self._wait_all(lambda index, status: not status & SW_SETPOINT_ACK,
                           state_timeouts, "清除旧应答")
            LOG.info("共同时间 %.6fs，量化后预计到达时差 %.6fs", plan.duration_s,
                     plan.estimated_arrival_spread_s)
            self._broadcast_armed = True
            self._broadcast(stages[0][1] | CW_NEW_SETPOINT)
            self._wait_all(lambda index, status: bool(status & SW_SETPOINT_ACK), state_timeouts, "新指令应答")
            self._broadcast(stages[0][1])
            self._wait_all(lambda index, status: not status & SW_SETPOINT_ACK, state_timeouts, "应答复位")
            arrivals = [None, None, None]
            positions = list(starts)

            def reached(index: int, status: int) -> bool:
                """同时检查到位位与本次绝对终点，记录首次观察到到位的主机时间。"""
                positions[index] = self.bus.axes[index].read(Object.ACTUAL_POSITION)
                tolerance = self.settings.axes[index].settings.position_tolerance_inc
                done = (bool(status & SW_TARGET_REACHED)
                        and abs(positions[index] - stages[index][0]) <= tolerance)
                if done and arrivals[index] is None:
                    arrivals[index] = self.clock()
                return done

            self._wait_all(reached, [a.settings.motion_timeout_s for a in self.settings.axes], "三轴到位")
            moving_arrivals = [arrival for index, arrival in enumerate(arrivals)
                               if stages[index][0] != starts[index]]
            spread = max(moving_arrivals) - min(moving_arrivals) if moving_arrivals else 0.0
            return GroupResult(tuple(positions), plan, spread)
        except BaseException as exc:
            primary = exc
            raise
        finally:
            errors = []
            try:
                self.halt_all()
            except Exception as exc:
                errors.append(str(exc))
            if not errors:
                try:
                    self.bus.restore()
                except Exception as exc:
                    errors.append(str(exc))
            else:
                errors.append("未确认全组停稳，保留临时 RPDO 映射，不执行恢复")
            if errors:
                message = "三轴清理失败: " + "; ".join(errors)
                if primary is None:
                    raise GroupError(message)
                LOG.error("%s；原错误：%s", message, primary)
