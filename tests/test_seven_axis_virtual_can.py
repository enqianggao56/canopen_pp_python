"""七轴真实EDS/SDO/RPDO/TPDO协议回归，仅使用python-can virtual。"""

import unittest
from contextlib import ExitStack
from dataclasses import replace
from unittest.mock import patch

from kinco_pp.objects import CW_HALT, Object
from kinco_pp.reciprocate import RunParameters
from kinco_pp.robstride import load_seven_axis_profiles
from kinco_pp.rs06 import RS06_QUICK_STOP
from kinco_pp.seven_axis_reciprocate import SevenAxisBus, SevenAxisReciprocationRunner
from kinco_pp.transport import TransportError
from test_four_axis_virtual_can import MixedAxisVirtualFixture


class SevenAxisVirtualCanTests(MixedAxisVirtualFixture, unittest.TestCase):
    """七个独立SDO服务和同步PDO缓冲，确认最后新增轴也进入联停与恢复。"""

    kinco_config_path = 'config/seven_axis.kinco.example.json'

    def companion_configs(self, channel):
        """使用原始共享EDS；仅虚拟从站缩短请求间隔和稳定窗口以加速协议测试。"""
        return tuple(replace(p, interface='virtual', channel=channel, synchronous_pdo_verified=True,
            sdo_pause_s=0.0001, poll_interval_s=0.005, standstill_duration_s=0.01,
            position_window_time_ms=10) for p in load_seven_axis_profiles('config/seven_axis.robstride.example.json'))

    def make_bus(self):
        """在与七个从站共享的virtual通道创建真实CANopen主站。"""
        return SevenAxisBus(self.settings, self.profiles)

    def runner(self, legs=1):
        """返回真实传输层上的七轴往返运行器。"""
        return SevenAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=legs), self.profiles)

    def snapshot(self):
        """只读记录全部临时PDO槽位、通信参数及SYNC，验证失败/正常退出的恢复。"""
        rows = []
        for io in self.bus.axes + self.bus.robstride_ios:
            objects = [(0x1005, 0), (0x1006, 0)]
            for index in io.node.object_dictionary:
                if index in (0x1400, 0x1403, 0x1800, 0x1801):
                    subindices = (1, 2) if index < 0x1800 else (1, 2, 3, 5, 6)
                    objects.extend((index, sub) for sub in subindices
                                   if sub in io.node.object_dictionary[index])
                    count = io.read(index + 0x200, 0)
                    objects.extend((index + 0x200, sub) for sub in range(count + 1))
            rows.append(tuple((index, sub, io.read(index, sub)) for index, sub in objects))
        return rows

    def assert_all_stopped(self):
        """检查全部七轴收到厂家对应停止字，周期SYNC任务已退出。"""
        self.assertTrue(all(sim.values[Object.CONTROLWORD] & CW_HALT for sim in self.sims[:3]))
        self.assertEqual([sim.values[Object.CONTROLWORD] for sim in self.sims[3:]], [RS06_QUICK_STOP]*4)
        self.assertFalse(self.bus._cyclic_sync_active)

    def test_seven_nodes_share_one_sync_and_use_only_tpdo_motion_feedback(self):
        """禁止七轴运动反馈SDO仍可完整往返；每次释放包含全部七个节点。"""
        self.assertEqual(len(self.bus.verify_identities()), 7)
        originals = self.snapshot()
        def guarded(io):
            """运动反馈只能走TPDO，配置和停稳确认允许SDO。"""
            variable = io._variable
            def access(index, subindex):
                if self.bus._cyclic_sync_active and index in (Object.STATUSWORD, Object.MODE_DISPLAY,
                        Object.ACTUAL_POSITION, 0x6063, Object.ACTUAL_VELOCITY):
                    raise RuntimeError('七轴运动反馈禁止SDO')
                return variable(index, subindex)
            return access
        with ExitStack() as stack:
            for io in self.bus.axes + self.bus.robstride_ios:
                stack.enter_context(patch.object(io, '_variable', side_effect=guarded(io)))
            report = self.runner(legs=2).run()
        self.assertTrue(report['success'])
        self.assertEqual([nodes for nodes, _ in self.releases], [tuple(range(1, 8))]*2)
        self.assertEqual([row['completed_round_trips'] for row in report['robstride_counts']], [1]*4)
        for node in (4, 5, 6, 7):
            self.assertEqual({comm for source, comm, _ in self.feedback_frames if source == node}, {0x1800, 0x1801})
        self.assertEqual(self.snapshot(), originals)
        self.assert_all_stopped()

    def test_last_node_tpdo_mapping_failure_rolls_back_all_seven(self):
        """最后节点TPDO写入中断时，前六轴及节点7部分配置恢复，尚未使能。"""
        originals = self.snapshot()
        self.reject_mapping_node, self.reject_rs_tpdo_mapping = 7, True
        with self.assertRaises(TransportError):
            self.runner().run()
        self.assertEqual(self.snapshot(), originals)
        self.assertFalse(self.releases)
        self.assertFalse(any(index == Object.CONTROLWORD for sim in self.sims for index, _, _ in sim.writes))
        self.assertFalse(self.bus.feedback.changed)

    def test_node_six_missing_speed_stops_all_seven(self):
        """中间EL05仅丢速度TPDO也联停七轴，并恢复全部PDO。"""
        originals = self.snapshot()
        self.drop_feedback_channel = (6, 0x1801)
        with self.assertRaisesRegex(Exception, '节点6.*TPDO'):
            self.runner().run()
        self.assert_all_stopped()
        self.assertEqual(self.snapshot(), originals)

    def test_last_node_target_send_failure_never_releases_partial_group(self):
        """节点7目标帧发送失败后不能发运动SYNC；已缓存的其他目标也不能被停止释放。"""
        send = self.bus._send
        def broken(cob_id, data):
            """仅拒绝最后轴的目标PDO，初始化及必要停止通道仍可用。"""
            if cob_id == self.profiles[-1].target_cob_id:
                raise TransportError('节点7目标发送失败')
            send(cob_id, data)
        with patch.object(self.bus, '_send', side_effect=broken):
            with self.assertRaisesRegex(Exception, '节点7目标发送失败'):
                self.runner().run()
        self.assertFalse(self.releases)
        self.assert_all_stopped()
        self.assertEqual([sim.values[Object.ACTUAL_POSITION] for sim in self.sims], [0]*7)

    def test_new_axis_identity_mismatch_prevents_configuration(self):
        """新节点身份不匹配时整组只读失败，不能继续配置其他轴。"""
        self.bus.extra_ios[-1].settings = replace(self.profiles[-1], expected_vendor_id=1)
        with self.assertRaisesRegex(TransportError, '身份不匹配'):
            self.bus.verify_identities()
        self.assertFalse(any(sim.writes for sim in self.sims))
        self.assertFalse(self.releases)

    def test_seven_axis_check_is_read_only(self):
        """七轴只读检查覆盖全部端点及共享通道，不发NMT、PDO或控制字。"""
        originals = self.snapshot()
        self.bus.verify_identities()
        self.bus.check_sync_configuration()
        checked = self.runner().check()
        self.assertEqual([row['node_id'] for row in checked['robstride_axes']], [4, 5, 6, 7])
        self.assertEqual(self.snapshot(), originals)
        self.assertFalse(any(sim.writes for sim in self.sims))
        self.assertFalse(self.feedback_frames)


if __name__ == '__main__':
    unittest.main()
