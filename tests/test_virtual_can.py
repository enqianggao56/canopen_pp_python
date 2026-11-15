"""通过 python-can virtual 总线和 CANopen LocalNode 验证真实 SDO 编解码。"""

import unittest
import uuid
from dataclasses import replace

import canopen

from kinco_pp.controller import ProfilePositionController
from kinco_pp.model import Motion, Settings
from kinco_pp.objects import Object
from kinco_pp.simulation import SimulatedDrive
from kinco_pp.transport import CanopenTransport, TransportError


class VirtualCanTests(unittest.TestCase):
    """使用两端独立的虚拟 CAN 网络；不会打开 SocketCAN 或 CANable。"""

    def setUp(self):
        """创建本地 CANopen 从站，并以回调把对象访问接到模拟状态机。"""
        self.settings = replace(Settings.load('config/servo.example.json'),
                                interface='virtual', channel=str(uuid.uuid4()), sdo_timeout_s=0.2)
        self.server = canopen.Network()
        self.server.connect(interface='virtual', channel=self.settings.channel)
        self.addCleanup(self.server.disconnect)
        self.node = canopen.LocalNode(self.settings.node_id, self.settings.eds)
        self.server.add_node(self.node)
        self.sim = SimulatedDrive()

        def read(index, subindex, od):
            """返回模拟状态及明确的身份值；其他对象交由 EDS 默认值处理。"""
            if index == Object.IDENTITY:
                return {0: 4, 1: 768, 2: 17988, 3: 0, 4: 123}[subindex]
            return self.sim.values.get(index) if index != Object.STATUSWORD else self.sim.read(index)

        def write(index, subindex, od, data):
            """使用实际 EDS 类型解码主站数据，并交给模拟驱动器。"""
            self.sim.write(index, od.decode_raw(data), subindex)

        self.node.add_read_callback(read)
        self.node.add_write_callback(write)
        self.client = CanopenTransport(self.settings)
        self.client.open()
        self.addCleanup(self.client.close)

    def test_real_sdo_identity_mode_and_signed_target(self):
        """走实际协议链路验证身份记录、int8 模式、int32 负目标及 uint32 速度。"""
        identity = self.client.verify_identity()
        self.assertEqual(identity['serial_number'], 123)
        self.assertEqual(self.client.node.sdo.MAX_RETRIES, 1)
        self.client.start()
        controller = ProfilePositionController(self.client, self.settings)
        controller.prepare()
        self.assertEqual(controller.move(Motion(-1000, 1, 1, 1)), -1000)
        self.assertEqual(self.sim.values[Object.PROFILE_VELOCITY], self.settings.velocity_raw(1))
        controller.halt()

    def test_identity_mismatch_prevents_control_writes(self):
        """配置与设备身份不一致时不得发送控制字。"""
        self.client.settings = replace(self.settings, expected_product_code=123)
        with self.assertRaisesRegex(TransportError, '身份不匹配'):
            self.client.verify_identity()
        self.assertEqual(self.sim.writes, [])

    def test_bad_object_error_identifies_index(self):
        """访问错误必须报告索引，便于现场定位配置问题。"""
        with self.assertRaisesRegex(TransportError, '0x1234:00'):
            self.client.read(0x1234)

    def test_unresponsive_node_reports_sdo_timeout(self):
        """模拟从站停止应答时，真实 SDO 通道必须有界失败。"""
        self.server.unsubscribe(0x600 + self.settings.node_id)
        with self.assertRaisesRegex(TransportError, '0x6041:00'):
            self.client.read(Object.STATUSWORD)


if __name__ == '__main__':
    unittest.main()
