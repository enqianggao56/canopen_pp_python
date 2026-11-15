"""三轴共享虚拟 CAN 集成测试；真实 SDO 编解码与单帧 RPDO，绝不连接电机。"""

import unittest
import uuid
from dataclasses import replace
from unittest.mock import patch

import canopen

from kinco_pp.group_controller import ThreeAxisController
from kinco_pp.group_model import GroupSettings
from kinco_pp.group_transport import (
    CanopenGroupBus, PDO_CONTROLWORD_MAPPING, PDO_DISABLED,
)
from kinco_pp.objects import CW_HALT, CW_NEW_SETPOINT, Object
from kinco_pp.simulation import SimulatedDrive
from kinco_pp.transport import TransportError


class GroupVirtualCanTests(unittest.TestCase):
    """一条 virtual 总线上建立三个 LocalNode 与一个共享主站连接。"""

    def setUp(self):
        """搭建协议从站，并保存非空的原始 RPDO 映射，用于检查完整恢复。"""
        source = GroupSettings.load('config/three_axis.example.json')
        channel = str(uuid.uuid4())
        self.settings = replace(source, pdo_configuration_verified=True, axes=tuple(
            replace(axis, settings=replace(axis.settings, interface='virtual', channel=channel,
                                           sdo_timeout_s=0.2)) for axis in source.axes))
        self.server = canopen.Network()
        self.server.connect(interface='virtual', channel=channel)
        self.addCleanup(self.server.disconnect)
        self.locals = []
        self.sims = []
        self.frames = []
        self.triggers = []
        self.reject_node = None
        self.comm_index = 0x1403
        self.map_index = 0x1603
        for axis in self.settings.axes:
            local = canopen.LocalNode(axis.settings.node_id, axis.settings.eds)
            self.server.add_node(local)
            local.set_data(self.map_index, 0, (1).to_bytes(1, 'little'))
            local.set_data(self.map_index, 1, (0x607A0020).to_bytes(4, 'little'))
            sim = SimulatedDrive()
            local.add_read_callback(self._reader(sim))
            local.add_write_callback(self._writer(local, sim))
            self.locals.append(local)
            self.sims.append(sim)
        self.server.subscribe(self.settings.shared_cob_id, self._receive_rpdo)
        self.bus = CanopenGroupBus(self.settings)
        self.bus.open()
        self.addCleanup(self.bus.close)

    def _reader(self, sim):
        """创建仅为电机对象提供动态值的 SDO 回调，PDO 配置交由 LocalNode 存储。"""
        def read(index, subindex, od):
            """实际身份提供三个条目，状态/位置经独立模拟器计算。"""
            if index == Object.IDENTITY:
                if subindex > 3:
                    raise canopen.sdo.SdoAbortedError(0x06090011)
                return {0: 3, 1: 768, 2: 17988, 3: 0}[subindex]
            if index == Object.STATUSWORD:
                return sim.read(index)
            if index >= 0x6000:
                return sim.values.get(index)
            return None
        return read

    def _writer(self, local, sim):
        """创建 SDO 写回调，检查 PDO 禁用顺序并可注入第二轴映射失败。"""
        def write(index, subindex, od, data):
            """对象通过实际 EDS 编解码；映射仍启用时的修改模拟驱动器拒绝。"""
            if index == self.map_index:
                cob_id = int.from_bytes(local.get_data(self.comm_index, 1), 'little')
                if not cob_id & PDO_DISABLED:
                    raise canopen.sdo.SdoAbortedError(0x06040043)
                if self.reject_node == local.id and subindex == 1:
                    self.reject_node = None
                    raise canopen.sdo.SdoAbortedError(0x06040041)
            if index >= 0x6000:
                sim.write(index, od.decode_raw(data), subindex)
        return write

    def _receive_rpdo(self, can_id, data, timestamp):
        """模拟从站根据实际 SDO 配好的 COB-ID、映射及 NMT 状态收取共享帧。"""
        self.frames.append(bytes(data))
        for local, sim in zip(self.locals, self.sims):
            cob_id = int.from_bytes(local.get_data(self.comm_index, 1), 'little')
            count = int.from_bytes(local.get_data(self.map_index, 0), 'little')
            mapping = int.from_bytes(local.get_data(self.map_index, 1), 'little')
            if (cob_id != can_id or count != 1 or mapping != PDO_CONTROLWORD_MAPPING
                    or local.nmt.state != 'OPERATIONAL'):
                continue
            word = int.from_bytes(data, 'little')
            if word & CW_NEW_SETPOINT:
                self.triggers.append((local.id, timestamp))
            sim.write(Object.CONTROLWORD, word)

    def test_three_nodes_receive_one_start_frame_and_restore_mapping(self):
        """实际协议验证三轴从同一帧触发，负目标、时间规划和旧映射完整恢复。"""
        self.assertEqual(len(self.bus.verify_identities()), 3)
        snapshots = [self.bus._snapshot(axis) for axis in self.bus.axes]
        self.assertIs(self.bus.axes[0].network, self.bus.axes[2].network)
        result = ThreeAxisController(self.bus, self.settings).run()
        self.assertEqual(result.positions, (1000, 2000, -1500))
        self.assertEqual(sum(bool(int.from_bytes(frame, 'little') & CW_NEW_SETPOINT) for frame in self.frames), 1)
        self.assertEqual([node for node, _ in self.triggers], [1, 2, 3])
        self.assertEqual(len({timestamp for _, timestamp in self.triggers}), 1)
        self.assertTrue(all(sim.values[Object.CONTROLWORD] & CW_HALT for sim in self.sims))
        self.assertEqual([self.bus._snapshot(axis) for axis in self.bus.axes], snapshots)

    def test_partial_mapping_failure_restores_changed_axes_without_enabling(self):
        """第二轴映射拒绝时撤销第一、二轴变更，不使能也不发出任何组触发。"""
        snapshots = [self.bus._snapshot(axis) for axis in self.bus.axes]
        self.reject_node = 2
        with self.assertRaises(TransportError):
            ThreeAxisController(self.bus, self.settings).run()
        self.assertEqual(self.frames, [])
        self.assertTrue(all(not sim.writes for sim in self.sims))
        self.assertEqual([self.bus._snapshot(axis) for axis in self.bus.axes], snapshots)

    def test_conflicting_tpdo_id_prevents_mapping_changes(self):
        """已有 TPDO 占用共享 COB-ID 时，在任何映射写入之前拒绝配置。"""
        self.locals[1].set_data(0x1800, 1, self.settings.shared_cob_id.to_bytes(4, 'little'))
        before = [self.bus._snapshot(axis) for axis in self.bus.axes]
        with self.assertRaisesRegex(TransportError, '已有其他 PDO'):
            self.bus.configure()
        self.assertEqual([self.bus._snapshot(axis) for axis in self.bus.axes], before)

    def test_mapping_requires_explicit_channel_verification(self):
        """即便直接调用底层 API，也不能忽略 PDO 通道核对标志。"""
        self.bus.settings = replace(self.settings, pdo_configuration_verified=False)
        with self.assertRaisesRegex(TransportError, 'pdo_configuration_verified'):
            self.bus.configure()
        self.assertEqual(self.bus._changed, [])

    def test_missing_third_node_fails_identity_check_without_control_writes(self):
        """第三节点掉线时在组准备之前失败，不能先启动前两轴。"""
        self.server.unsubscribe(0x603)
        with self.assertRaises(TransportError):
            self.bus.verify_identities()
        self.assertTrue(all(not sim.writes for sim in self.sims))
        self.assertEqual(self.frames, [])

    def test_closing_borrowed_axis_does_not_disconnect_other_axes(self):
        """单轴包装器不拥有共享网络，关闭它不应让另外两轴掉线。"""
        self.bus.axes[0].close()
        self.assertIsNotNone(self.bus.network.bus)
        self.assertEqual(self.bus.axes[1].read(Object.IDENTITY, 1), 768)

    def test_restore_continues_after_one_axis_restore_error(self):
        """恢复一轴失败仍要恢复其余轴，并报告未恢复的配置。"""
        originals = [self.bus._snapshot(axis) for axis in self.bus.axes]
        self.bus.configure()
        restore = self.bus._write_mapping
        def fail_second(axis, *mapping):
            """仅模拟第二轴无法写回原映射。"""
            if axis.settings.node_id == 2:
                raise TransportError('restore disconnected')
            restore(axis, *mapping)
        with patch.object(self.bus, '_write_mapping', side_effect=fail_second):
            with self.assertRaisesRegex(TransportError, '节点 2'):
                self.bus.restore()
        self.assertEqual(self.bus._snapshot(self.bus.axes[0]), originals[0])
        self.assertEqual(self.bus._snapshot(self.bus.axes[2]), originals[2])
        self.assertEqual(len(self.bus._changed), 1)
        self.bus.restore()


if __name__ == '__main__':
    unittest.main()
