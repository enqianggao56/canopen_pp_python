"""混合四轴回归：厂家单位、固定端点、共同释放、到位屏障及异常联停；不访问硬件。"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from kinco_pp.four_axis_reciprocate import (FourAxisReciprocationRunner,
                                           SimulatedFourAxisBus, main, validate_topology)
from kinco_pp.group_controller import GroupError
from kinco_pp.group_model import GroupSettings
from kinco_pp.objects import CW_HALT, CW_NEW_SETPOINT, Object, SW_FAULT, SW_TARGET_REACHED
from kinco_pp.reciprocate import RunParameters
from kinco_pp.rs06 import (RS06Settings, RS06Peer, RS06_QUICK_STOP, SimulatedRS06,
                           TORQUE_LIMIT, WATCHDOG)
from test_reciprocate import Clock


class FourAxisTests(unittest.TestCase):
    """使用独立的 Kinco/RS06 模拟器和虚拟时钟，禁止自动运动。"""

    def setUp(self):
        """使用600圈三轴与100圈第四轴，均固定从0开始。"""
        self.settings = GroupSettings.load('config/reciprocate.example.json')
        self.rs06 = RS06Settings.load('config/rs06.example.json')
        self.bus = SimulatedFourAxisBus()
        self.clock = Clock()

    def runner(self, parameters=None, sink=None):
        """返回共用虚拟时钟的四轴运行器。"""
        return FourAxisReciprocationRunner(self.bus, self.settings,
            parameters or RunParameters(legs=2), self.rs06,
            self.clock, self.clock.sleep, sink)

    def test_joint_round_trip_has_vendor_units_and_one_release_per_leg(self):
        """前三轴600圈，第四轴100圈；第四轴不收Kinco握手字或0x6084。"""
        report = self.runner().run()
        self.assertEqual([event[2] for event in self.bus.events], [1638400, 0])
        self.assertEqual([event[1] for event in self.bus.events], [(39321600,)*3, (0,)*3])
        writes = self.bus.rs06_io.writes
        self.assertIn((Object.PROFILE_VELOCITY, 1000, 0), writes)
        self.assertIn((Object.PROFILE_ACCELERATION, 500, 0), writes)
        self.assertIn((TORQUE_LIMIT, 100, 0), writes)
        self.assertIn((WATCHDOG, 20000, 1), writes)
        self.assertFalse(any(index in (Object.HALT_OPTION, Object.PROFILE_DECELERATION) for index, _, _ in writes))
        self.assertFalse(any(value & (CW_NEW_SETPOINT | CW_HALT) for index, value, _ in writes
                             if index == Object.CONTROLWORD))
        self.assertEqual(report['completed_round_trips'], 1)
        self.assertEqual(report['rs06_completed_round_trips'], 1)
        self.assertEqual(len(report['final_states']), 4)
        self.assertLess(report['four_axis_model_arrival_spread_s'], 0.001)
        self.assertEqual(len(report['four_axis_recent_coordination']), 2)
        self.assertFalse(report['hardware_validated'])
        self.assertTrue(self.bus.restored)

    def test_check_only_reads_and_current_position_does_not_reanchor(self):
        """重启保留第四轴0至100圈区间；检查不能写目标或状态。"""
        self.bus = SimulatedFourAxisBus((100000,)*3, 10000)
        checked = self.runner().check()
        self.assertEqual(checked['rs06']['origin_position_inc'], 0)
        self.assertEqual(checked['rs06']['far_position_inc'], 1638400)
        self.assertTrue(all(not io.writes for io in self.bus.axes + [self.bus.rs06_io]))
        self.assertFalse(self.bus.frames)

    def test_recovery_and_full_rs06_legs_have_separate_counts(self):
        """中途恢复后两程才构成完整100圈往返；不能将恢复段记为完整单程。"""
        self.bus = SimulatedFourAxisBus((100000,)*3, 10000)
        report = self.runner(RunParameters(legs=3)).run()
        self.assertEqual(report['rs06_completed_legs'], 3)
        self.assertEqual(report['rs06_completed_full_legs'], 2)
        self.assertEqual(report['rs06_completed_round_trips'], 1)
        self.assertEqual(self.bus.events[-1][2], 1638400)

    def test_rs06_already_at_selected_target_does_not_count_zero_leg(self):
        """第四轴已在远端时等待三轴首程，零距离保持不能计为完成100圈。"""
        self.bus = SimulatedFourAxisBus(rs06_position=1638400)
        report = self.runner().run()
        self.assertEqual(report['rs06_completed_legs'], 1)
        self.assertEqual(report['rs06_completed_full_legs'], 1)

    def test_reverse_restart_preserves_both_fixed_endpoints(self):
        """中途指定reverse先回原起点，后续仍使用原远端。"""
        self.bus = SimulatedFourAxisBus((10000,)*3, 20000)
        report = self.runner(RunParameters(legs=2, start_direction='reverse')).run()
        self.assertEqual([event[2] for event in self.bus.events], [0, 1638400])
        self.assertEqual(report['rs06_completed_full_legs'], 1)

    def test_stale_reached_bit_cannot_finish_rs06_at_wrong_position(self):
        """旧bit10与错误位置不能过到位屏障，超时后四轴联停。"""
        read = self.bus.rs06_io.read
        def stale(index, subindex=0):
            """目标开始后伪造旧到位位，实际位置始终在起点。"""
            if self.bus.events and index == Object.ACTUAL_POSITION:
                return 0
            if self.bus.events and index == Object.STATUSWORD:
                return read(index, subindex) | SW_TARGET_REACHED
            return read(index, subindex)
        self.rs06 = replace(self.rs06, motion_timeout_s=63)
        with patch.object(self.bus.rs06_io, 'read', side_effect=stale):
            with self.assertRaisesRegex(Exception, 'RS06|rs06'):
                self.runner().run()
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
        self.assertEqual(self.bus.rs06_io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
        self.assertEqual(len(self.bus.events), 1)

    def test_rs06_fault_mode_loss_and_range_violation_stop_kinco(self):
        """第四轴各类异常都阻止反向程，并请求前三轴暂停。"""
        for failure in ('fault', 'mode', 'range', 'read'):
            with self.subTest(failure=failure):
                self.bus = SimulatedFourAxisBus()
                read = self.bus.rs06_io.read
                def broken(index, subindex=0):
                    """只在共同触发后注入异常，确保检验运动中停止路径。"""
                    if self.bus.events:
                        if failure == 'fault' and index == Object.STATUSWORD:
                            return read(index, subindex) | SW_FAULT
                        if failure == 'mode' and index == Object.MODE_DISPLAY:
                            return 8
                        if failure == 'range' and index == Object.ACTUAL_POSITION:
                            return self.rs06.max_position_inc + 1
                        if failure == 'read' and index == Object.MODE_DISPLAY:
                            raise RuntimeError('RS06 SDO timeout')
                    return read(index, subindex)
                with patch.object(self.bus.rs06_io, 'read', side_effect=broken):
                    with self.assertRaises(Exception):
                        self.runner().run()
                self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))
                self.assertEqual(self.bus.rs06_io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
                self.assertEqual(len(self.bus.events), 1)

    def test_kinco_fault_requests_rs06_stop_before_waiting_for_axes(self):
        """任意Kinco故障也必须先发出第四轴停止请求，再等待各轴静止。"""
        runner = self.runner()
        original = self.bus.axes[0].read
        def faulty(index, subindex=0):
            """在首次共同触发后报告故障。"""
            if self.bus.events and index == Object.STATUSWORD:
                return original(index, subindex) | SW_FAULT
            return original(index, subindex)
        requests = []
        halt = runner.peer.request_halt
        def request():
            """记录第四轴停止请求先于任何轴的停稳确认。"""
            requests.append('rs_stop')
            halt()
        with patch.object(self.bus.axes[0], 'read', side_effect=faulty), \
                patch.object(runner.peer, 'request_halt', side_effect=request):
            with self.assertRaises(GroupError):
                runner.run()
        self.assertTrue(requests)
        self.assertEqual(self.bus.rs06_io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes))

    def test_failed_first_halt_still_requests_other_axes(self):
        """Kinco第一轴停止通信失败也不能跳过RS06；不得恢复未确认停稳的映射。"""
        write = self.bus.axes[0].write
        def broken(index, value, subindex=0):
            """共同触发后拒绝第一轴Halt，其他写入正常。"""
            if self.bus.events and index == Object.CONTROLWORD and value & CW_HALT:
                raise RuntimeError('first halt failed')
            write(index, value, subindex)
        with patch.object(self.bus.axes[0], 'write', side_effect=broken):
            with self.assertRaises(GroupError):
                self.runner().run()
        self.assertEqual(self.bus.rs06_io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
        self.assertTrue(all(io.values[Object.CONTROLWORD] & CW_HALT for io in self.bus.axes[1:]))
        self.assertFalse(self.bus.restored)

    def test_duration_and_interrupt_stop_four_axes(self):
        """限时与Ctrl+C均完成四轴停止；截止前未触发时不补发运动。"""
        report = self.runner(RunParameters(duration_s=0.1)).run()
        self.assertEqual(report['stop_reason'], 'duration')
        self.assertEqual(self.bus.rs06_io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
        self.assertFalse(self.bus.events)
        self.bus = SimulatedFourAxisBus()
        def interrupt(row):
            """首次共同启动后模拟用户中断。"""
            if self.bus.events:
                raise KeyboardInterrupt()
        report = self.runner(RunParameters(sample_interval_s=0.01), interrupt).run()
        self.assertEqual(report['stop_reason'], 'interrupt')
        self.assertEqual(self.bus.rs06_io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
        self.assertTrue(self.bus.restored)

    def test_reverse_waits_until_fourth_axis_reaches_target(self):
        """前三轴先到位时保持等待，第四轴在第10秒到位前不能启动反程。"""
        read = self.bus.rs06_io.read
        broadcast = self.bus.broadcast
        def delayed(index, subindex=0):
            """首程将第四轴实际位置保持在起点至虚拟10秒。"""
            if len(self.bus.events) == 1 and self.clock() < 10 and index == Object.ACTUAL_POSITION:
                return 0
            return read(index, subindex)
        def checked(word):
            """检查反程共同触发时第四轴已经满足到位屏障。"""
            if len(self.bus.events) == 1 and word & CW_NEW_SETPOINT:
                self.assertGreaterEqual(self.clock(), 10)
            broadcast(word)
        with patch.object(self.bus.rs06_io, 'read', side_effect=delayed), \
                patch.object(self.bus, 'broadcast', side_effect=checked):
            report = self.runner().run()
        self.assertEqual(report['completed_round_trips'], 1)

    def test_invalid_fourth_axis_prevents_all_writes(self):
        """第四轴越界、已有运动、完整行程超时都在任何使能前拒绝。"""
        for failure in ('position', 'active', 'full_timeout'):
            with self.subTest(failure=failure):
                self.bus = SimulatedFourAxisBus()
                self.rs06 = RS06Settings.load('config/rs06.example.json')
                if failure == 'position':
                    self.bus.rs06_io.values[Object.ACTUAL_POSITION] = -100
                elif failure == 'active':
                    self.bus.rs06_io.values[Object.STATUSWORD] = 0x27
                else:
                    self.bus = SimulatedFourAxisBus((39321500,)*3, 1638300)
                    self.rs06 = replace(self.rs06, motion_timeout_s=20)
                with self.assertRaises(Exception):
                    self.runner().run()
                self.assertTrue(all(not io.writes for io in self.bus.axes + [self.bus.rs06_io]))

    def test_rs06_protocol_units_and_configuration_validation(self):
        """厂家公式独立于Kinco；100RPM和2秒对应1000/500，反馈1000对应100RPM。"""
        self.assertEqual(self.rs06.velocity_raw(100), 1000)
        self.assertEqual(self.rs06.acceleration_raw(100 / 60 / 2), 500)
        self.assertEqual(self.rs06.feedback_velocity_rpm(-1000), -100)
        for kwargs in ({'velocity_rpm': 101}, {'velocity_rpm': float('nan')},
                       {'far_position_inc': 0}, {'torque_limit_raw': 0},
                       {'hardware_verified': 1}, {'standstill_duration_s': 5}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                replace(self.rs06, **kwargs)
        for kwargs in ({'node_id': 1}, {'channel': 'can1'}, {'target_cob_id': 0x500}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                validate_topology(self.settings, replace(self.rs06, **kwargs))

    def test_feedback_stream_contains_four_vendor_correct_speeds(self):
        """CSV和内存采样保持四轴配置顺序，并使用各厂家自己的反馈换算。"""
        rows = []
        report = self.runner(sink=rows.append).run()
        self.assertTrue(rows)
        self.assertTrue(all(len(row['velocities_rpm']) == 4 for row in rows))
        self.assertEqual(len(report['final_states']), 4)

    def test_unverified_execute_does_not_open_hardware(self):
        """默认示例的四轴实机执行必须在连接前拒绝。"""
        with patch('kinco_pp.four_axis_reciprocate.FourAxisBus') as factory:
            self.assertEqual(main(['--execute', '--legs', '2']), 1)
            factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
