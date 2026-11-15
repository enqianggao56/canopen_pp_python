"""用虚拟时间验证 PP 状态、握手、到位、故障及参数约束，不连接电机。"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from kinco_pp.cli import main
from kinco_pp.controller import DriveError, DriveTimeout, ProfilePositionController
from kinco_pp.model import Motion, Settings
from kinco_pp.objects import (
    CW_HALT, CW_NEW_SETPOINT, Object, SW_FAULT, SW_FOLLOWING_ERROR,
    SW_LIMIT_ACTIVE, SW_SETPOINT_ACK, SW_TARGET_REACHED,
)
from kinco_pp.simulation import SimulatedDrive


class FakeClock:
    """通过 sleep 累加时间，避免超时测试实际等待。"""

    def __init__(self):
        """初始化虚拟秒数。"""
        self.now = 0.0

    def __call__(self):
        """返回当前虚拟时间，单位秒。"""
        return self.now

    def sleep(self, seconds):
        """将虚拟时钟前移 seconds 秒。"""
        self.now += seconds


class ControllerTests(unittest.TestCase):
    """覆盖成功定位及不能被忽略的失败分支。"""

    def setUp(self):
        """为每个测试创建独立模拟从站与短超时配置。"""
        self.settings = replace(Settings.load('config/servo.example.json'),
                                state_timeout_s=0.1, motion_timeout_s=0.1)
        self.io = SimulatedDrive(initial_position=100)
        clock = FakeClock()
        self.controller = ProfilePositionController(self.io, self.settings, clock, clock.sleep)
        self.motion = Motion(1000, 1, 1, 1)

    def test_relative_move_twice_has_one_trigger_per_request(self):
        """相对指令必须各产生一次上升沿，并累加到不同终点。"""
        self.controller.prepare()
        self.assertEqual(self.controller.move(self.motion), 1100)
        self.assertEqual(self.controller.move(self.motion), 2100)
        triggers = [v for i, v, _ in self.io.writes if i == Object.CONTROLWORD and v & CW_NEW_SETPOINT]
        self.assertEqual(triggers, [0x5F, 0x5F])
        self.controller.halt()
        self.assertTrue(self.io.values[Object.CONTROLWORD] & CW_HALT)
        self.assertEqual(self.io.values[Object.STATUSWORD] & 0x6F, 0x27)

    def test_absolute_negative_target(self):
        """确认原点后支持 int32 负坐标，采用手册绝对定位控制字。"""
        self.controller.settings = replace(self.settings, absolute_reference_verified=True)
        self.controller.prepare()
        self.assertEqual(self.controller.move(Motion(-500, 1, 1, 1, relative=False)), -500)
        self.assertIn((Object.CONTROLWORD, 0x3F, 0), self.io.writes)

    def test_reuse_halted_enabled_drive_without_shutdown(self):
        """重复运行应保留锁轴，不能先下发 shutdown 松轴。"""
        self.controller.prepare()
        self.controller.move(self.motion)
        self.controller.halt()
        self.io.writes.clear()
        self.controller.prepare()
        self.assertFalse(any(i == Object.CONTROLWORD and v & 0xF == 6
                             for i, v, _ in self.io.writes))
        self.assertEqual(self.controller.move(self.motion), 2100)

    def test_refuse_takeover_of_active_enabled_drive(self):
        """已使能但未暂停的驱动器不能被自动接管。"""
        self.controller.prepare()
        self.io.values[Object.CONTROLWORD] = 0x0F
        self.io.writes.clear()
        with self.assertRaisesRegex(DriveError, '拒绝接管'):
            self.controller.prepare()
        self.assertEqual(self.io.writes, [])

    def test_halt_waits_for_actual_zero_velocity(self):
        """暂停不能只看旧到位位，实际速度非零时必须超时报错。"""
        self.controller.prepare()
        read = self.io.read
        with patch.object(self.io, 'read', side_effect=lambda i, s=0: 1 if i == Object.ACTUAL_VELOCITY else read(i, s)):
            with self.assertRaises(DriveTimeout):
                self.controller.halt()

    def test_zero_distance_completes_without_needing_bit10_falling_edge(self):
        """零位移也执行握手，不强求观察到转瞬即逝的到位位清零。"""
        self.controller.prepare()
        self.assertEqual(self.controller.move(Motion(0, 1, 1, 1)), 100)

    def test_standstill_accepts_velocity_noise_only_with_stable_position(self):
        """位置稳定时容许已配置的速度反馈噪声；持续漂移则不能判断停稳。"""
        self.controller.settings = replace(self.settings, zero_velocity_tolerance_rpm=0.5)
        self.controller.prepare()
        read = self.io.read
        with patch.object(self.io, 'read', side_effect=lambda i, s=0: 5000 if i == Object.ACTUAL_VELOCITY else read(i, s)):
            self.controller.halt()
        self.controller.prepare()
        def drifting(index, subindex=0):
            """模拟速度值虽很小，但位置持续移动的异常情况。"""
            if index == Object.ACTUAL_POSITION:
                self.io.values[index] += 11
            return read(index, subindex)
        with patch.object(self.io, 'read', side_effect=drifting), self.assertRaises(DriveTimeout):
            self.controller.halt()

    def test_no_move_before_prepare(self):
        """未使能调用必须在写入前拒绝。"""
        with self.assertRaises(DriveError):
            self.controller.move(self.motion)
        self.assertEqual(self.io.writes, [])

    def test_invalid_parameters_and_limits_do_not_write_motion(self):
        """起终点、速度和绝对原点约束均在目标写入前验证。"""
        self.controller.prepare()
        for motion in [Motion(100000, 1, 1, 1), Motion(1000, 31, 1, 1),
                       Motion(1000, 1, 11, 1), Motion(1000, 1, 1, 1, False),
                       Motion(True, 1, 1, 1), Motion(1000, float('nan'), 1, 1)]:
            with self.subTest(motion=motion), self.assertRaises(ValueError):
                self.controller.move(motion)
        self.assertNotIn(Object.TARGET_POSITION, [i for i, _, _ in self.io.writes])

    def test_fault_limit_and_following_error_fail_before_enabling(self):
        """危险状态不应通过写控制字隐式清除。"""
        for flag in [SW_FAULT, SW_LIMIT_ACTIVE, SW_FOLLOWING_ERROR]:
            self.io.values[Object.STATUSWORD] = flag
            with self.subTest(flag=flag), self.assertRaises(DriveError):
                self.controller.prepare()
        self.assertEqual(self.io.writes, [])

    def test_mode_confirmation_times_out(self):
        """使能后实际模式仍未切换时不得触发目标，准备阶段始终保持 Halt。"""
        read = self.io.read
        with patch.object(self.io, 'read', side_effect=lambda i, s=0: 0 if i == Object.MODE_DISPLAY else read(i, s)):
            with self.assertRaises(DriveTimeout):
                self.controller.prepare()
        controls = [v for i, v, _ in self.io.writes if i == Object.CONTROLWORD]
        self.assertTrue(all(v & CW_HALT for v in controls))
        self.assertFalse(any(v & CW_NEW_SETPOINT for v in controls))

    def test_mode_display_can_update_only_after_enable(self):
        """兼容只在进入 operation enabled 后更新实际模式的固件。"""
        read = self.io.read
        def delayed(index, subindex=0):
            """未使能时模式显示为零，使能后才显示写入的 PP 模式。"""
            if index == Object.MODE_DISPLAY and self.io.values[Object.STATUSWORD] & 0x6F != 0x27:
                return 0
            return read(index, subindex)
        with patch.object(self.io, 'read', side_effect=delayed):
            self.controller.prepare()
            self.assertEqual(self.controller.move(self.motion), 1100)

    def test_missing_or_stale_ack_times_out(self):
        """bit12 卡高或永不置高时均须停止等待，卡高时不得触发新目标。"""
        self.controller.prepare()
        read = self.io.read
        for stuck_high in [True, False]:
            def altered(index, subindex=0):
                """将状态字的应答位固定到测试值。"""
                value = read(index, subindex)
                if index == Object.STATUSWORD:
                    return value | SW_SETPOINT_ACK if stuck_high else value & ~SW_SETPOINT_ACK
                return value
            with self.subTest(high=stuck_high), patch.object(self.io, 'read', side_effect=altered):
                before = len(self.io.writes)
                with self.assertRaises(DriveTimeout):
                    self.controller.move(self.motion)
                if stuck_high:
                    self.assertFalse(any(i == Object.TARGET_POSITION for i, _, _ in self.io.writes[before:]))

    def test_stale_target_reached_with_wrong_position_does_not_complete(self):
        """旧 bit10 即便一直置位，也不能让错误位置被判断成到位。"""
        self.controller.prepare()
        read = self.io.read
        def stale(index, subindex=0):
            """固定实际位置，并始终返回到位标志。"""
            value = read(index, subindex)
            if index == Object.ACTUAL_POSITION:
                return 100
            return value | SW_TARGET_REACHED if index == Object.STATUSWORD else value
        with patch.object(self.io, 'read', side_effect=stale), self.assertRaises(DriveTimeout):
            self.controller.move(self.motion)

    def test_state_loss_during_motion_is_an_error(self):
        """握手完成后驱动器退出使能，不应仅等待到位超时。"""
        self.controller.prepare()
        read = self.io.read
        def disabled(index, subindex=0):
            """握手复位后模拟失去使能。"""
            value = read(index, subindex)
            if index == Object.STATUSWORD and self.io.remaining_polls <= 2 and self.io.target != 100:
                return (value & ~0x6F) | 0x21
            return value
        with patch.object(self.io, 'read', side_effect=disabled), self.assertRaisesRegex(DriveError, '退出使能'):
            self.controller.move(self.motion)

    def test_unit_conversion_matches_manual_table_9_11(self):
        """使用手册报文的数值校验换算，不能将 RPM 直接写入 0x6081。"""
        self.assertEqual(self.settings.velocity_raw(200), 0x369D03)
        self.assertEqual(self.settings.acceleration_raw(100), 0x1A36E)
        for value in [0, -1, float('inf'), 1e-15, 1e30]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.settings.velocity_raw(value)
        with self.assertRaises(ValueError):
            replace(self.settings, node_id=True)
        with self.assertRaises(ValueError):
            replace(self.settings, hardware_verified='false')

    def test_cli_execute_requires_verified_config_before_connecting(self):
        """示例配置不能误触发实机连接。"""
        with patch('kinco_pp.cli.CanopenTransport') as transport:
            self.assertEqual(main(['--execute']), 1)
            transport.assert_not_called()

    def test_cli_failure_and_interrupt_both_attempt_halt(self):
        """异常和 Ctrl+C 均通过 finally 暂停，且退出码反映失败。"""
        for error, expected in [(RuntimeError('test failure'), 1), (KeyboardInterrupt(), 130)]:
            io = SimulatedDrive()
            with self.subTest(error=error), patch('kinco_pp.cli.SimulatedDrive', return_value=io), \
                    patch.object(ProfilePositionController, 'move', side_effect=error):
                self.assertEqual(main([]), expected)
            self.assertTrue(io.values[Object.CONTROLWORD] & CW_HALT)


if __name__ == '__main__':
    unittest.main()
