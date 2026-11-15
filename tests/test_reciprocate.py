"""往返程序回归：同步方向切换、固定端点、限时/中断联停及参数校验。"""

import unittest
from unittest.mock import patch

from kinco_pp.group_controller import GroupError
from kinco_pp.group_model import GroupSettings
from kinco_pp.group_transport import SimulatedGroupBus
from kinco_pp.objects import CW_HALT, CW_NEW_SETPOINT, Object
from kinco_pp.reciprocate import (
    DurationReached, ReciprocationRunner, RunParameters, main,
)


class Clock:
    """测试专用虚拟秒时钟，sleep 仅前移时间。"""

    def __init__(self):
        """从零开始计时。"""
        self.now = 0.0

    def __call__(self):
        """返回当前虚拟秒数。"""
        return self.now

    def sleep(self, seconds):
        """推进虚拟时间，避免超时测试实际等候。"""
        self.now += seconds


class ReciprocationTests(unittest.TestCase):
    """所有用例均使用独立模拟器，不会访问 CANable。"""

    def setUp(self):
        """建立600圈、600RPM、2秒加减速的测试设置与共享模拟总线。"""
        self.settings = GroupSettings.load('config/reciprocate.example.json')
        self.bus = SimulatedGroupBus()
        self.clock = Clock()

    def runner(self, parameters=None, sink=None):
        """构造复用本测试总线与时钟的往返运行器。"""
        return ReciprocationRunner(self.bus, self.settings,
            parameters or RunParameters(legs=2), self.clock, self.clock.sleep, sink)

    def test_acceleration_time_converts_to_five_rps2(self):
        """600RPM=10rps，2秒加减速应得到5rps/s，而非直接将2写入对象。"""
        params = RunParameters()
        self.assertEqual(params.acceleration_rps2, 5)
        self.assertEqual(params.deceleration_rps2, 5)
        self.assertEqual(self.settings.axes[0].settings.acceleration_raw(5), 5368)

    def test_four_legs_complete_two_round_trips_all_axes_same_direction(self):
        """四程应为正、反、正、反，三轴同向，每程只发送一次共同启动帧。"""
        runner = self.runner(RunParameters(legs=4))
        report = runner.run()
        self.assertEqual(report['completed_round_trips'], 2)
        self.assertEqual(report['completed_legs'], 4)
        self.assertEqual([leg['phase'] for leg in report['recent_legs']],
                         ['forward', 'reverse', 'forward', 'reverse'])
        self.assertEqual([leg['positions_inc'] for leg in report['recent_legs']],
                         [(39321600,)*3, (0,)*3, (39321600,)*3, (0,)*3])
        self.assertEqual(sum(bool(frame & CW_NEW_SETPOINT) for frame in self.bus.frames), 4)
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertTrue(self.bus.restored)

    def test_anchor_prevents_feedback_error_accumulating(self):
        """每程后反馈偏移3inc，下一程应校正到固定端点，不能逐圈累计漂移。"""
        restore = self.bus.restore
        def drift():
            """模拟停稳复查时的小幅位置读数变化。"""
            restore()
            for io in self.bus.axes:
                io.values[Object.ACTUAL_POSITION] -= 3
        with patch.object(self.bus, 'restore', side_effect=drift):
            report = self.runner(RunParameters(legs=4)).run()
        self.assertEqual(report['recent_legs'][-1]['positions_inc'], (0, 0, 0))
        self.assertEqual([row['position_inc'] for row in report['final_states']], [-3, -3, -3])

    def test_duration_in_motion_stops_all_and_marks_incomplete_leg(self):
        """观察截止时正在运动也要中止等待、暂停三轴，并保持完成程数为零。"""
        read = self.bus.axes[0].read
        runner = self.runner(RunParameters(duration_s=1))
        with patch.object(self.bus.axes[0], 'read', side_effect=lambda i, s=0: 0 if i == Object.ACTUAL_POSITION else read(i, s)):
            report = runner.run()
        self.assertEqual(report['stop_reason'], 'duration')
        self.assertEqual(report['completed_legs'], 0)
        self.assertGreaterEqual(report['elapsed_s'], 1)
        self.assertLess(report['elapsed_s'], 2)
        self.assertTrue(report['success'])
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertTrue(self.bus.restored)

    def test_deadline_before_trigger_does_not_start_motion(self):
        """在配置阶段耗尽时间后，不能为了完成一程而继续触发。"""
        configure = self.bus.configure
        def delayed():
            """模拟配置耗时超过观察窗口。"""
            configure()
            self.clock.sleep(2)
        with patch.object(self.bus, 'configure', side_effect=delayed):
            report = self.runner(RunParameters(duration_s=1)).run()
        self.assertEqual(report['stop_reason'], 'duration')
        self.assertFalse(any(frame & CW_NEW_SETPOINT for frame in self.bus.frames))

    def test_duration_cleanup_failure_is_not_reported_as_success(self):
        """限时停止时广播失败，即使其余暂停继续尝试，也必须报告清理异常。"""
        read = self.bus.axes[0].read
        broadcast = self.bus.broadcast
        def broken(value):
            """首次触发后使共同Halt发送失败，检验异常传播与逐轴回退。"""
            if value & CW_HALT and any(frame & CW_NEW_SETPOINT for frame in self.bus.frames):
                raise RuntimeError('halt broadcast failed')
            broadcast(value)
        runner = self.runner(RunParameters(duration_s=1))
        with patch.object(self.bus.axes[0], 'read', side_effect=lambda i, s=0: 0 if i == Object.ACTUAL_POSITION else read(i, s)), \
                patch.object(self.bus, 'broadcast', side_effect=broken):
            with self.assertRaisesRegex(GroupError, '三轴清理失败'):
                runner.run()
        self.assertFalse(runner.report['success'])
        self.assertEqual(runner.report['stop_reason'], 'error')
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))

    def test_ctrl_c_attempts_group_halt_and_stops_loop(self):
        """采样期间Ctrl+C走正常组清理，不应启动后续反程。"""
        def interrupted(row):
            """仅在共同触发后中断。"""
            if self.bus.frames:
                raise KeyboardInterrupt()
        runner = self.runner(RunParameters(sample_interval_s=0.01), sink=interrupted)
        report = runner.run()
        self.assertEqual(report['stop_reason'], 'interrupt')
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertEqual(sum(bool(frame & CW_NEW_SETPOINT) for frame in self.bus.frames), 1)

    def test_limits_reject_whole_loop_before_any_enabling(self):
        """6000圈超出示例范围，不能先运行去程再发现回程或累计位置不可行。"""
        with self.assertRaises(ValueError):
            self.runner(RunParameters(turns=6000, legs=2)).run()
        self.assertTrue(all(not io.writes for io in self.bus.axes))
        self.assertEqual(self.bus.frames, [])

    def test_sample_sink_receives_configured_order(self):
        """反馈在同一线程送到sink，并包含三个轴和方向信息。"""
        rows = []
        report = self.runner(sink=rows.append).run()
        self.assertGreater(len(rows), 0)
        self.assertEqual(report['sample_count'], len(rows))
        self.assertEqual(len(rows[0]['positions_inc']), 3)
        self.assertEqual(len(rows[0]['velocities_rpm']), 3)

    def test_invalid_parameters_and_unverified_execute_are_rejected(self):
        """拒绝非有限值、零加速时间及未经核对的实机执行。"""
        for kwargs in [dict(acceleration_time_s=0), dict(velocity_rpm=float('nan')),
                       dict(duration_s=-1), dict(legs=0), dict(turns=True)]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RunParameters(**kwargs)
        with patch('kinco_pp.reciprocate.CanopenGroupBus') as bus:
            self.assertEqual(main(['--execute', '--duration', '120']), 1)
            bus.assert_not_called()


if __name__ == '__main__':
    unittest.main()
