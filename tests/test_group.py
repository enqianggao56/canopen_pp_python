"""三轴规划、单帧触发及故障联停测试；所有设备均为纯软件模拟。"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from kinco_pp.group_cli import main
from kinco_pp.group_controller import GroupError, ThreeAxisController
from kinco_pp.group_model import GroupSettings, plan_group, profile_time
from kinco_pp.group_transport import SimulatedGroupBus
from kinco_pp.model import Motion
from kinco_pp.objects import CW_HALT, CW_NEW_SETPOINT, Object, SW_FAULT, SW_SETPOINT_ACK


class Clock:
    """可推进的虚拟时钟，避免故障测试实际等待。"""

    def __init__(self):
        """初始化虚拟秒数。"""
        self.now = 0.0

    def __call__(self):
        """返回当前虚拟秒数。"""
        return self.now

    def sleep(self, seconds):
        """前移指定秒数。"""
        self.now += seconds


class GroupTests(unittest.TestCase):
    """验证三轴共享启动以及任何退出路径都处理其他轴。"""

    def setUp(self):
        """读取三轴示例，构造共享模拟总线和短状态超时。"""
        settings = GroupSettings.load('config/three_axis.example.json')
        self.settings = replace(settings, axes=tuple(
            replace(axis, settings=replace(axis.settings, state_timeout_s=0.2, motion_timeout_s=3))
            for axis in settings.axes))
        self.bus = SimulatedGroupBus()
        clock = Clock()
        self.controller = ThreeAxisController(self.bus, self.settings, clock, clock.sleep)

    def test_common_frame_after_all_axes_staged(self):
        """三个目标全部装载后只能出现一次共同启动帧，最终均到达并暂停。"""
        broadcast = self.bus.broadcast
        def guarded(value):
            """触发时检查所有轴都已装载目标且保持 Halt。"""
            if value & CW_NEW_SETPOINT:
                for io in self.bus.axes:
                    self.assertIn(Object.TARGET_POSITION, io.values)
                    self.assertIn(Object.PROFILE_VELOCITY, io.values)
                    self.assertTrue(io.values[Object.CONTROLWORD] & CW_HALT)
            broadcast(value)
        with patch.object(self.bus, 'broadcast', side_effect=guarded):
            result = self.controller.run()
        self.assertEqual(result.positions, (1000, 2000, -1500))
        self.assertEqual([v for v in self.bus.frames if v & CW_NEW_SETPOINT], [0x5F])
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertTrue(self.bus.restored)

    def test_repeat_run_does_not_duplicate_previous_relative_command(self):
        """重复调用每次只多一帧上升沿，绝不重放前次相对目标。"""
        self.controller.run()
        result = self.controller.run()
        self.assertEqual(result.positions, (2000, 4000, -3000))
        self.assertEqual(sum(bool(v & CW_NEW_SETPOINT) for v in self.bus.frames), 2)

    def test_one_invalid_axis_prevents_all_enabling(self):
        """第三轴目标越界也必须在第一轴使能之前拒绝整组。"""
        bad = replace(self.settings.axes[2], motion=Motion(200000, 1, 1, 1))
        self.controller.settings = replace(self.settings, axes=self.settings.axes[:2] + (bad,))
        with self.assertRaises(ValueError):
            self.controller.run()
        self.assertEqual(self.bus.frames, [])
        self.assertTrue(all(not io.writes for io in self.bus.axes))

    def test_missing_ack_halts_all_without_retry(self):
        """一个轴不应答时停止三轴，不重发相对位置触发。"""
        read = self.bus.axes[1].read
        with patch.object(self.bus.axes[1], 'read', side_effect=lambda i, s=0: read(i, s) & ~SW_SETPOINT_ACK if i == Object.STATUSWORD else read(i, s)):
            with self.assertRaisesRegex(GroupError, '轴 y'):
                self.controller.run()
        self.assertEqual(sum(bool(v & CW_NEW_SETPOINT) for v in self.bus.frames), 1)
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))

    def test_partial_prepare_failure_never_broadcasts_enable(self):
        """第二轴准备失败时，不应广播使能给还未设置 PP 的第三轴。"""
        read = self.bus.axes[1].read
        with patch.object(self.bus.axes[1], 'read', side_effect=lambda i, s=0: 0 if i == Object.MODE_DISPLAY else read(i, s)):
            with self.assertRaisesRegex(GroupError, '轴 y'):
                self.controller.run()
        self.assertEqual(self.bus.frames, [])
        self.assertEqual(self.bus.axes[2].writes, [])
        self.assertTrue(self.bus.restored)

    def test_arrival_timeout_stops_axes_that_already_arrived(self):
        """一轴长期停留在错误位置时，已经到位的轴也必须走全组暂停路径。"""
        read = self.bus.axes[1].read
        with patch.object(self.bus.axes[1], 'read', side_effect=lambda i, s=0: 0 if i == Object.ACTUAL_POSITION else read(i, s)):
            with self.assertRaisesRegex(GroupError, '三轴到位'):
                self.controller.run()
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertTrue(self.bus.restored)

    def test_fault_during_motion_requests_halt_on_other_axes(self):
        """中途故障即便无法确认故障轴停稳，也要暂停其余轴，并保留映射。"""
        read = self.bus.axes[1].read
        def fault(index, subindex=0):
            """触发后持续报告故障，防止清理阶段假装故障消失。"""
            value = read(index, subindex)
            if index == Object.STATUSWORD and any(v & CW_NEW_SETPOINT for v in self.bus.frames):
                value |= SW_FAULT
            return value
        with patch.object(self.bus.axes[1], 'read', side_effect=fault):
            with self.assertRaisesRegex(GroupError, '轴 y'):
                self.controller.run()
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertFalse(self.bus.restored)

    def test_stop_write_failure_does_not_skip_remaining_axes(self):
        """第一轴无法收取暂停时，仍尝试第二、第三轴，而不是在第一个异常返回。"""
        write = self.bus.axes[0].write
        def broken(index, value, subindex=0):
            """运动触发后拒绝第一轴的 Halt 写入。"""
            if index == Object.CONTROLWORD and value & CW_HALT and any(v & CW_NEW_SETPOINT for v in self.bus.frames):
                raise RuntimeError('node disconnected')
            write(index, value, subindex)
        with patch.object(self.bus.axes[0], 'write', side_effect=broken):
            with self.assertRaisesRegex(GroupError, '全组已确认停稳且临时 PDO 已恢复'):
                self.controller.run()
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes[1:]))
        self.assertTrue(self.bus.restored)

    def test_keyboard_interrupt_requests_all_halts(self):
        """用户中断也要经完整的全组暂停和映射恢复路径。"""
        read = self.bus.axes[1].read
        raised = False
        def interrupted(index, subindex=0):
            """只在触发后中断一次，允许清理继续执行。"""
            nonlocal raised
            if index == Object.STATUSWORD and not raised and self.bus.frames:
                raised = True
                raise KeyboardInterrupt()
            return read(index, subindex)
        with patch.object(self.bus.axes[1], 'read', side_effect=interrupted), self.assertRaises(KeyboardInterrupt):
            self.controller.run()
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertTrue(self.bus.restored)

    def test_absolute_and_zero_distance_axes(self):
        """绝对定位也使用共享控制字；零位移轴保持当前位置。"""
        axes = tuple(replace(axis, settings=replace(axis.settings, absolute_reference_verified=True),
                             motion=replace(axis.motion, target_inc=target, relative=False))
                     for axis, target in zip(self.settings.axes, (0, 1000, -2000)))
        self.controller.settings = replace(self.settings, axes=axes)
        self.controller.controllers = [type(controller)(io, axis.settings, self.controller.clock, self.controller.sleep)
                                       for controller, io, axis in zip(self.controller.controllers, self.bus.axes, axes)]
        result = self.controller.run()
        self.assertEqual(result.positions, (0, 1000, -2000))
        self.assertIn(0x3F, self.bus.frames)

    def test_all_zero_targets_do_not_enable_or_trigger(self):
        """三轴均无位移时应直接返回，不能为了同步而使能电机。"""
        self.controller.settings = replace(self.settings, axes=tuple(
            replace(axis, motion=replace(axis.motion, target_inc=0)) for axis in self.settings.axes))
        self.assertEqual(self.controller.run().positions, (0, 0, 0))
        self.assertEqual(self.bus.frames, [])
        self.assertTrue(all(not io.writes for io in self.bus.axes))

    def test_cli_requires_real_hardware_and_pdo_confirmation(self):
        """示例配置不能通过 --execute 误接入真实 CAN。"""
        with patch('kinco_pp.group_cli.CanopenGroupBus') as bus:
            self.assertEqual(main(['--execute']), 1)
            bus.assert_not_called()

    def test_inspect_never_configures_pdo_and_reports_close_failure(self):
        """只读入口不得配置 PDO，且不能把关闭连接失败误报成成功。"""
        with patch('kinco_pp.group_cli.CanopenGroupBus') as factory:
            bus = factory.return_value
            bus.axes = self.bus.axes
            bus.verify_identities.return_value = ({}, {}, {})
            bus.close.side_effect = RuntimeError('close failed')
            self.assertEqual(main(['--inspect']), 1)
            bus.configure.assert_not_called()
            bus.start_all.assert_not_called()


class PlannerTests(unittest.TestCase):
    """验证梯形、三角轮廓及同步时间缩放，不依赖模拟驱动器推进规则。"""

    def setUp(self):
        """构造一、四、九圈的已知梯形轮廓。"""
        source = GroupSettings.load('config/three_axis.example.json')
        self.settings = replace(source, axes=tuple(
            replace(axis, settings=replace(axis.settings, min_position_inc=-1000000,
                                           max_position_inc=1000000, max_velocity_rpm=60),
                    motion=Motion(turns * 65536, 60, 1, 1))
            for axis, turns in zip(source.axes, (1, 4, 9))))

    def test_profile_known_trapezoid_and_triangle(self):
        """一圈 a=d=1、峰值 1rps 应耗时 2s；0.01 圈三角轮廓为 0.2s。"""
        self.assertAlmostEqual(profile_time(1, 1, 1, 1)[0], 2)
        self.assertAlmostEqual(profile_time(0.01, 1, 1, 1)[0], 0.2)

    def test_short_axes_scaled_to_longest_time(self):
        """九圈轴约 10s，其余轴缩放后在整数参数精度下接近共同到达时间。"""
        plan = plan_group(self.settings, (0, 0, 0))
        self.assertAlmostEqual(plan.duration_s, 10, delta=0.001)
        self.assertAlmostEqual(plan.axes[0].motion.acceleration_rps2, 0.04, delta=0.002)
        self.assertTrue(all(axis.motion.velocity_rpm <= 60 for axis in plan.axes))
        self.assertTrue(all(abs(axis.estimated_duration_s - 10) < 0.005 for axis in plan.axes))

    def test_longer_duration_and_unachievable_duration(self):
        """可指定更长时间，但不能靠超过原速度/加速度上限压缩时间。"""
        plan = plan_group(replace(self.settings, duration_s=20), (0, 0, 0))
        self.assertAlmostEqual(plan.duration_s, 20)
        self.assertAlmostEqual(plan.axes[2].motion.velocity_rpm, 30, delta=0.1)
        with self.assertRaises(ValueError):
            plan_group(replace(self.settings, duration_s=5), (0, 0, 0))

    def test_unrepresentable_slow_profile_is_rejected(self):
        """极端时间缩放导致速度量化为零时，即使允许很长超时也必须拒绝。"""
        settings = replace(self.settings, duration_s=1e9, axes=tuple(
            replace(axis, settings=replace(axis.settings, motion_timeout_s=2e9)) for axis in self.settings.axes))
        with self.assertRaises(ValueError):
            plan_group(settings, (0, 0, 0))

    def test_duplicate_nodes_mixed_bus_and_mixed_position_modes_rejected(self):
        """不允许节点重复、跨总线或共享字无法表达的混合定位方式。"""
        axes = self.settings.axes
        variants = [replace(axes[2], settings=replace(axes[2].settings, node_id=1)),
                    replace(axes[2], settings=replace(axes[2].settings, channel='can1')),
                    replace(axes[2], motion=replace(axes[2].motion, relative=False))]
        for bad in variants:
            with self.subTest(axis=bad), self.assertRaises(ValueError):
                replace(self.settings, axes=axes[:2] + (bad,))


if __name__ == '__main__':
    unittest.main()
