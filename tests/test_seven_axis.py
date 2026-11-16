"""七轴纯模拟回归：型号单位、固定端点、到位屏障及异常联停；不访问硬件。"""

import unittest
from dataclasses import replace
from unittest.mock import patch

from kinco_pp.group_controller import GroupError
from kinco_pp.group_model import GroupSettings
from kinco_pp.objects import CW_HALT, CW_NEW_SETPOINT, Object, SW_FAULT
from kinco_pp.reciprocate import RunParameters
from kinco_pp.robstride import load_seven_axis_profiles, validate_profiles
from kinco_pp.rs06 import RS06_QUICK_STOP
from kinco_pp.seven_axis_reciprocate import (SevenAxisBus, SevenAxisReciprocationRunner,
                                            SimulatedSevenAxisBus, main)
from test_reciprocate import Clock


class SevenAxisTests(unittest.TestCase):
    """以虚拟时钟验证三台Kinco与四台RobStride的独立控制和共同时间规划。"""

    def setUp(self):
        """读取独立七轴模板，仅模拟时确认Kinco坐标。"""
        self.settings = GroupSettings.load('config/seven_axis.kinco.example.json')
        self.settings = replace(self.settings, axes=tuple(replace(axis,
            settings=replace(axis.settings, absolute_reference_verified=True)) for axis in self.settings.axes))
        self.profiles = load_seven_axis_profiles('config/seven_axis.robstride.example.json')
        self.clock = Clock()
        self.bus = SimulatedSevenAxisBus()

    def runner(self, parameters=None, sink=None):
        """返回使用同一虚拟时钟的七轴运行器。"""
        return SevenAxisReciprocationRunner(self.bus, self.settings, parameters or RunParameters(legs=2),
            self.profiles, self.clock, self.clock.sleep, sink)

    def ios(self):
        """返回节点1至7的独立模拟IO。"""
        return self.bus.axes + [self.bus.rs06_io] + self.bus.extra_ios

    def test_models_share_motion_units_but_not_torque_conversion(self):
        """新增轴100圈/100RPM与RS06一致；力矩百分比按各自手册换算。"""
        self.assertEqual([(p.node_id, p.model) for p in self.profiles],
                         [(4, 'RS06'), (5, 'RS00'), (6, 'EL05'), (7, 'EL05')])
        self.assertEqual([p.torque_limit_nm for p in self.profiles], [1.1, 0.5, 0.2, 0.2])
        for p in self.profiles:
            self.assertEqual(p.far_position_inc - p.origin_position_inc, 1638400)
            self.assertEqual(p.velocity_raw(p.velocity_rpm), 1000)
            self.assertEqual(p.acceleration_raw(p.velocity_rpm / 60 / p.acceleration_time_s), 500)
            self.assertFalse(p.hardware_verified)

    def test_seven_axis_round_trip_and_feedback_order(self):
        """七轴共同往返，新增轴采用绝对位置PP且各自计数、反馈、停止。"""
        report = self.runner().run()
        self.assertEqual(self.bus.companion_events, [(1638400,)*4, (0,)*4])
        self.assertEqual([row['node_id'] for row in report['final_states']], list(range(1, 8)))
        self.assertEqual([row['completed_round_trips'] for row in report['robstride_counts']], [1]*4)
        self.assertEqual(report['axis_count'], 7)
        self.assertLess(report['axis_model_arrival_spread_s'], 0.001)
        self.assertEqual(len(report['axis_recent_coordination']), 2)
        self.assertTrue(all(len(row['positions_inc']) == 7 for row in report['samples']))
        for io in self.ios()[3:]:
            self.assertEqual(io.values[Object.CONTROLWORD], RS06_QUICK_STOP)
            self.assertIn((Object.PROFILE_VELOCITY, 1000, 0), io.writes)
            self.assertFalse(any(index == Object.PROFILE_DECELERATION for index, _, _ in io.writes))
        self.assertTrue(self.bus.restored)

    def test_seventh_axis_delays_reversal_until_arrival(self):
        """其他六轴先到位时等待最后EL05，不能提前反向。"""
        io = self.bus.extra_ios[-1]
        read, broadcast = io.read, self.bus.broadcast
        def delayed(index, subindex=0):
            """首程第10秒前保持第七轴实际位置在起点。"""
            if len(self.bus.events) == 1 and self.clock() < 10 and index == Object.ACTUAL_POSITION:
                return 0
            return read(index, subindex)
        def checked(word):
            """第二次共同上升沿必须等第七轴满足到位屏障。"""
            if len(self.bus.events) == 1 and word & CW_NEW_SETPOINT:
                self.assertGreaterEqual(self.clock(), 10)
            broadcast(word)
        with patch.object(io, 'read', side_effect=delayed), patch.object(self.bus, 'broadcast', side_effect=checked):
            self.assertTrue(self.runner().run()['success'])

    def test_rs00_stop_failure_still_requests_both_el05(self):
        """节点5停止通信失败不能跳过节点6/7，也不能恢复未确认的映射。"""
        io = self.bus.extra_ios[0]
        write = io.write
        def broken(index, value, subindex=0):
            """触发后拒绝节点5停止字，其余操作正常。"""
            if self.bus.events and index == Object.CONTROLWORD and value == RS06_QUICK_STOP:
                raise RuntimeError('节点5停止失败')
            write(index, value, subindex)
        with patch.object(io, 'write', side_effect=broken):
            with self.assertRaisesRegex(GroupError, '节点5停止失败'):
                self.runner().run()
        self.assertTrue(all(axis.values[Object.CONTROLWORD] & CW_HALT for axis in self.bus.axes))
        self.assertEqual([axis.values[Object.CONTROLWORD] for axis in self.bus.extra_ios[1:]], [RS06_QUICK_STOP]*2)
        self.assertFalse(self.bus.restored)

    def test_last_axis_fault_stops_every_started_axis(self):
        """节点7运动期间故障进入全组停止，前三轴及其他伴随轴均收到停止。"""
        io = self.bus.extra_ios[-1]
        read = io.read
        def fault(index, subindex=0):
            """仅在共同启动后注入第七轴故障位。"""
            value = read(index, subindex)
            return value | SW_FAULT if self.bus.events and index == Object.STATUSWORD else value
        with patch.object(io, 'read', side_effect=fault):
            with self.assertRaisesRegex(Exception, 'el05/节点7'):
                self.runner().run()
        self.assertEqual([axis.values[Object.CONTROLWORD] for axis in self.ios()[3:]], [RS06_QUICK_STOP]*4)
        self.assertTrue(all(axis.values[Object.CONTROLWORD] & CW_HALT for axis in self.bus.axes))

    def test_recovery_uses_independent_fixed_origins(self):
        """不同绝对起点和中途重启不外移端点，恢复段不计作完整行程。"""
        origins = (100, 200, -300, 400)
        self.profiles = tuple(replace(p, origin_position_inc=origin, far_position_inc=origin+1638400,
            min_position_inc=origin-25, max_position_inc=origin+1638425) for p, origin in zip(self.profiles, origins))
        self.bus = SimulatedSevenAxisBus(companion_positions=(100, 10200, -300, 400))
        runner = self.runner(RunParameters(legs=3))
        checked = runner.check()
        self.assertEqual([row['origin_position_inc'] for row in checked['robstride_axes']], list(origins))
        self.assertTrue(all(not io.writes for io in self.ios()))
        self.assertTrue(checked['partial_first_leg'])
        report = runner.run()
        self.assertEqual(report['completed_full_legs'], 2)
        self.assertEqual([row['completed_full_legs'] for row in report['robstride_counts']], [3, 2, 3, 3])
        self.assertEqual(self.bus.companion_events[-1], tuple(origin+1638400 for origin in origins))

    def test_new_axis_invalid_before_any_write(self):
        """新增轴越界、完整程超时或已使能均须在全组使能前拒绝。"""
        for failure in ('position', 'timeout', 'enabled'):
            with self.subTest(failure=failure):
                self.setUp()
                if failure == 'position':
                    self.bus.extra_ios[1].values[Object.ACTUAL_POSITION] = -100
                elif failure == 'timeout':
                    self.profiles = self.profiles[:-1] + (replace(self.profiles[-1], motion_timeout_s=20),)
                else:
                    self.bus.extra_ios[-1].values[Object.STATUSWORD] = 0x27
                with self.assertRaises(Exception):
                    self.runner().run()
                self.assertTrue(all(not io.writes for io in self.ios()))

    def test_new_axis_topology_and_equal_motion_are_checked(self):
        """节点/COB-ID重复、不同总线、速度/位移不一致均被拒绝。"""
        for changes in ({'node_id': 4}, {'target_cob_id': 0x504}, {'channel': 'other'}):
            profiles = self.profiles[:1] + (replace(self.profiles[1], **changes),) + self.profiles[2:]
            with self.assertRaises(ValueError):
                SevenAxisBus(self.settings, profiles)
        for changes in ({'velocity_rpm': 50}, {'far_position_inc': 1600000}):
            profiles = self.profiles[:-1] + (replace(self.profiles[-1], **changes),)
            with self.assertRaises(ValueError):
                validate_profiles(profiles)

    def test_duration_stops_seven_axes(self):
        """限时提前结束也确认所有已使能轴停止。"""
        report = self.runner(RunParameters(duration_s=0.1)).run()
        self.assertEqual(report['stop_reason'], 'duration')
        self.assertEqual([axis.values[Object.CONTROLWORD] for axis in self.ios()[3:]], [RS06_QUICK_STOP]*4)

    def test_template_execute_never_opens_hardware(self):
        """七轴模板未确认时在连接前拒绝实机执行。"""
        with patch('kinco_pp.seven_axis_reciprocate.SevenAxisBus') as factory:
            self.assertEqual(main(['--execute', '--legs', '2']), 1)
            factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
