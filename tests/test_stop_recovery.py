"""验证停止超时重试顺序、次数及最终停稳与请求异常分离。"""
import unittest
from unittest.mock import Mock
from kinco_pp.group_controller import ThreeAxisController, GroupError
from kinco_pp.transport import read_stop_configuration, TransportError
from kinco_pp.group_model import GroupSettings
from kinco_pp.group_transport import SimulatedGroupBus


class SdoCommunicationError(Exception):
    """模拟通信库的无应答异常，避免测试依赖实机。"""


def timeout():
    """构造与传输层一致的异常原因链。"""
    error = TransportError('SDO timeout')
    error.__cause__ = SdoCommunicationError('No SDO response received')
    return error


class StopRecoveryTests(unittest.TestCase):
    """使用模拟IO检查恢复边界。"""

    def test_read_retry_bounded_and_no_retry_abort(self):
        """读超时最多三次；非超时立即传播。"""
        io = Mock()
        io.read.side_effect = [timeout(), timeout(), 254]
        self.assertEqual(read_stop_configuration(io, 0x1403, 2, lambda _: None), 254)
        self.assertEqual(io.read.call_count, 3)
        io.read.reset_mock()
        io.read.side_effect = timeout()
        with self.assertRaises(TransportError):
            read_stop_configuration(io, 0x1403, 2, lambda _: None)
        self.assertEqual(io.read.call_count, 3)
        io.read.reset_mock()
        io.read.side_effect = ValueError('abort')
        with self.assertRaises(ValueError):
            read_stop_configuration(io, 0x1403, 2, lambda _: None)
        self.assertEqual(io.read.call_count, 1)

    def test_retry_after_all_axes_and_preserve_error(self):
        """先覆盖全组后重试；最终静止与首次请求错误同时保留。"""
        c = ThreeAxisController(SimulatedGroupBus(), GroupSettings.load('config/three_axis.example.json'), sleep=lambda _: None)
        events = []
        def request(index):
            events.append(index)
            if index == 0 and events.count(0) == 1:
                raise timeout()
        c.controllers = [Mock() for _ in range(3)]
        for i, controller in enumerate(c.controllers):
            controller.request_halt.side_effect = lambda i=i: request(i)
        with self.assertRaises(GroupError):
            c.halt_all()
        self.assertEqual(events, [0, 1, 2, 0])
        self.assertTrue(c._all_halted)
        c.controllers[1].wait_halted.side_effect = RuntimeError('still moving')
        with self.assertRaises(GroupError):
            c.halt_all()
        self.assertFalse(c._all_halted)
        self.assertEqual(c.controllers[2].wait_halted.call_count, 2)

    def test_persistent_timeout_does_not_skip_peers(self):
        """持续无应答只补发两次，仍检查其余轴停稳。"""
        c = ThreeAxisController(SimulatedGroupBus(), GroupSettings.load('config/three_axis.example.json'), sleep=lambda _: None)
        c.controllers = [Mock() for _ in range(3)]
        c.controllers[0].request_halt.side_effect = timeout()
        c.controllers[0].wait_halted.side_effect = timeout()
        with self.assertRaises(GroupError):
            c.halt_all()
        self.assertEqual(c.controllers[0].request_halt.call_count, 3)
        self.assertFalse(c._all_halted)
        for controller in c.controllers[1:]:
            controller.request_halt.assert_called_once()
            controller.wait_halted.assert_called_once()
