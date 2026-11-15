"""不连接硬件的简化 PP 从站，仅验证主站流程，不模拟动力学或真实驱动器。"""

from .objects import (
    CW_ENABLE, CW_HALT, CW_NEW_SETPOINT, CW_RELATIVE, CW_SHUTDOWN,
    CW_SWITCH_ON, Object, PP_MODE, SW_SETPOINT_ACK, SW_TARGET_REACHED,
)


class SimulatedDrive:
    """模拟手册中的状态切换、bit12 应答和延迟到位；默认初始位置为零。"""

    def __init__(self, initial_position: int = 0):
        """设置初始位置 inc 和状态，无返回值；不进行任何 CAN 通信。"""
        self.values = {
            Object.STATUSWORD: 0x0070 | SW_TARGET_REACHED,
            Object.ACTUAL_POSITION: initial_position,
            Object.ACTUAL_VELOCITY: 0,
            Object.CONTROLWORD: 0,
            Object.MODE: 0,
            Object.MODE_DISPLAY: 0,
        }
        self.writes = []
        self.remaining_polls = 0
        self.target = initial_position

    def read(self, index: int, subindex: int = 0) -> int:
        """返回模拟对象整数；每次状态读取推进运动，不支持的对象抛 KeyError。"""
        if index == Object.STATUSWORD and self.remaining_polls:
            self.remaining_polls -= 1
            if self.remaining_polls == 0:
                self.values[Object.ACTUAL_POSITION] = self.target
                self.values[Object.ACTUAL_VELOCITY] = 0
                self.values[Object.STATUSWORD] |= SW_TARGET_REACHED
            elif self.remaining_polls < 4:
                self.values[Object.STATUSWORD] &= ~SW_TARGET_REACHED
        return self.values[index]

    def write(self, index: int, value: int, subindex: int = 0) -> None:
        """记录写入并处理模式、状态及新位置上升沿，无返回值。"""
        previous = self.values.get(index, 0)
        self.values[index] = value
        self.writes.append((index, value, subindex))
        if index == Object.MODE:
            self.values[Object.MODE_DISPLAY] = value
        if index != Object.CONTROLWORD:
            return
        state = {CW_SHUTDOWN: 0x0031, CW_SWITCH_ON: 0x0033, CW_ENABLE: 0x0037}.get(value & 0xF, 0x0070)
        self.values[Object.STATUSWORD] = (self.values[Object.STATUSWORD] & ~0x007F) | state
        if not value & CW_NEW_SETPOINT:
            self.values[Object.STATUSWORD] &= ~SW_SETPOINT_ACK
        if value & CW_HALT:
            self.remaining_polls = 0
            self.values[Object.ACTUAL_VELOCITY] = 0
            self.values[Object.STATUSWORD] |= SW_TARGET_REACHED
        elif value & CW_NEW_SETPOINT and not previous & CW_NEW_SETPOINT:
            if self.values[Object.MODE_DISPLAY] != PP_MODE or value & 0xF != CW_ENABLE:
                raise ValueError("模拟驱动器尚未使能 PP 模式")
            self.values[Object.STATUSWORD] |= SW_SETPOINT_ACK
            self.values[Object.ACTUAL_VELOCITY] = 1
            self.target = self.values[Object.TARGET_POSITION]
            if value & CW_RELATIVE:
                self.target += self.values[Object.ACTUAL_POSITION]
            # 故意保留第一轮旧到位位，验证主站还检查实际位置。
            self.remaining_polls = 5
