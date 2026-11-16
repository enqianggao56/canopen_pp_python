"""TPDO反馈缓存回归：使用真实RS06 EDS和可控时钟，不连接任何CAN接口。"""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import canopen

from kinco_pp.group_model import GroupSettings
from kinco_pp.objects import Object
from kinco_pp.pdo_feedback import CyclicFeedback
from kinco_pp.rs06 import RS06Settings
from kinco_pp.transport import TransportError
from test_reciprocate import Clock


class PdoFeedbackTests(unittest.TestCase):
    """独立验证有符号解码、两条反馈流时效、缓存重置与恢复失败隔离。"""

    def setUp(self):
        """由EDS默认值提供通信对象，直接派发TPDO回调以控制帧顺序和时间。"""
        self.clock = Clock()
        self.active = True
        local = canopen.LocalNode(4, 'RS06.eds')
        self.io = Mock()
        self.io.settings = RS06Settings()
        self.io.node = SimpleNamespace(object_dictionary=local.object_dictionary,
                                       nmt=SimpleNamespace(state='PRE-OPERATIONAL'))
        self.io.read.side_effect = lambda index, sub=0: int.from_bytes(local.get_data(index, sub), 'little')
        def write(index, value, sub=0):
            """按原始EDS类型写入本地对象，保持配置快照和映射长度可回读。"""
            variable = local.object_dictionary[index][sub]
            local.set_data(index, sub, variable.encode_raw(value))
        self.io.write.side_effect = write
        self.network = Mock()
        self.feedback = CyclicFeedback(self.network, [self.io], [self.io],
            GroupSettings.load('config/reciprocate.example.json'), lambda: self.active,
            clock=self.clock, sleep=self.clock.sleep, remap_state_axes=(self.io,))
        self.feedback.configure()
        self.callbacks = dict(self.feedback.subscriptions)
        self.read = self.io.feedback_provider

    def state(self, position=-123456, mode=1):
        """模拟节点4的七字节TPDO1，位置为inc，状态保持原始位值。"""
        payload = (0x8437).to_bytes(2, 'little') + position.to_bytes(4, 'little', signed=True)
        self.callbacks[0x184](0x184, payload + mode.to_bytes(1, 'little', signed=True), 0)

    def velocity(self, raw=-1000):
        """模拟节点4的四字节TPDO2，速度为0.1rpm原始值。"""
        self.callbacks[0x284](0x284, raw.to_bytes(4, 'little', signed=True), 0)

    def test_signed_feedback_and_vendor_units(self):
        """负位置、负速度、负模式正确解码；状态字不应符号扩展。"""
        self.state(mode=-1)
        self.velocity()
        self.assertEqual(self.read(Object.STATUSWORD, 0), 0x8437)
        self.assertEqual(self.read(Object.ACTUAL_POSITION, 0), -123456)
        self.assertEqual(self.read(Object.MODE_DISPLAY, 0), -1)
        self.assertEqual(self.io.settings.feedback_velocity_rpm(self.read(Object.ACTUAL_VELOCITY, 0)), -100)

    def test_streams_have_independent_expiry(self):
        """状态帧持续更新不能掩盖速度帧过期，反之亦然。"""
        self.state()
        self.velocity()
        self.clock.sleep(0.31)
        self.state()
        self.assertEqual(self.read(Object.ACTUAL_POSITION, 0), -123456)
        with self.assertRaisesRegex(TransportError, 'velocity_time'):
            self.read(Object.ACTUAL_VELOCITY, 0)
        self.clock.sleep(0.31)
        self.velocity()
        with self.assertRaisesRegex(TransportError, 'state_time'):
            self.read(Object.STATUSWORD, 0)

    def test_malformed_frames_do_not_refresh_feedback(self):
        """短帧和额外字节不匹配映射，不能刷新通信时效。"""
        self.state()
        self.velocity()
        self.clock.sleep(0.31)
        for cob, sizes in ((0x184, (6, 8)), (0x284, (3, 5))):
            for size in sizes:
                self.callbacks[cob](cob, bytes(size), 0)
        for index in (Object.STATUSWORD, Object.ACTUAL_VELOCITY):
            with self.assertRaisesRegex(TransportError, '过期'):
                self.read(index, 0)

    def test_reset_requires_new_frames_and_inactive_allows_sdo(self):
        """触发前旧缓存必须失效；停止后读取返回None，让通信层改用SDO复核。"""
        self.state()
        self.velocity()
        self.feedback.reset()
        with self.assertRaisesRegex(TransportError, '首帧TPDO'):
            self.read(Object.ACTUAL_POSITION, 0)
        self.active = False
        self.assertIsNone(self.read(Object.ACTUAL_POSITION, 0))
        self.assertIsNone(self.read(Object.ACTUAL_VELOCITY, 0))
        self.active = True
        self.state(position=987)
        self.assertEqual(self.read(Object.ACTUAL_POSITION, 0), 987)

    def test_restore_failure_does_not_skip_other_channel(self):
        """TPDO2恢复失败仍恢复TPDO1，失败快照保留且读取钩子移除。"""
        mapper = self.feedback.mappers[0x1801]
        mapper._write_mapping = Mock(side_effect=TransportError('模拟恢复失败'))
        self.active = False
        with self.assertRaisesRegex(TransportError, 'TPDO2恢复失败'):
            self.feedback.restore()
        self.assertEqual([comm for _, comm, _, _ in self.feedback.changed], [0x1801])
        self.assertEqual(self.io.read(0x1A00, 0), 2)
        self.assertIsNone(self.io.feedback_provider)
        self.assertFalse(self.feedback.subscriptions)


if __name__ == '__main__':
    unittest.main()
