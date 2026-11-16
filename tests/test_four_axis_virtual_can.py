"""四轴真实 EDS/SDO/同步 RPDO 协议回归；仅连接 python-can virtual，不连接 can0。"""

import unittest
import uuid
from dataclasses import replace

import canopen

from kinco_pp.four_axis_reciprocate import (FourAxisBus, FourAxisReciprocationRunner,
                                           SYNC_COB_ID, TARGET_POSITION_MAPPING)
from kinco_pp.group_model import GroupSettings
from kinco_pp.group_transport import PDO_CONTROLWORD_MAPPING, PDO_DISABLED
from kinco_pp.objects import CW_HALT, CW_NEW_SETPOINT, Object
from kinco_pp.reciprocate import RunParameters
from kinco_pp.rs06 import RS06Settings, SimulatedRS06, RS06_QUICK_STOP
from kinco_pp.simulation import SimulatedDrive
from kinco_pp.transport import TransportError


class FourAxisVirtualCanTests(unittest.TestCase):
    """四个LocalNode模拟驱动，实际SDO编码和RPDO缓冲于同一SYNC生效。"""

    def setUp(self):
        """读取两种原始EDS，创建共享虚拟网络和动态状态对象。"""
        channel = str(uuid.uuid4())
        source = GroupSettings.load('config/reciprocate.example.json')
        self.settings = replace(source, pdo_configuration_verified=True, axes=tuple(
            replace(axis, settings=replace(axis.settings, interface='virtual', channel=channel,
                                           sdo_timeout_s=0.2)) for axis in source.axes))
        self.rs06 = replace(RS06Settings.load('config/rs06.example.json'),
                            interface='virtual', channel=channel, synchronous_pdo_verified=True)
        self.server = canopen.Network()
        self.server.connect(interface='virtual', channel=channel)
        self.addCleanup(self.server.disconnect)
        self.locals, self.sims, self.pending, self.releases = [], [], {}, []
        self.reject_rs_mapping = False
        configs = [axis.settings for axis in self.settings.axes] + [self.rs06]
        for config in configs:
            local = canopen.LocalNode(config.node_id, config.eds)
            self.server.add_node(local)
            sim = SimulatedRS06() if config.node_id == self.rs06.node_id else SimulatedDrive()
            local.set_data(0x1603, 0, (1).to_bytes(1, 'little'))
            local.set_data(0x1603, 1, (PDO_CONTROLWORD_MAPPING).to_bytes(4, 'little'))
            local.add_read_callback(self._reader(sim))
            local.add_write_callback(self._writer(local, sim))
            self.locals.append(local)
            self.sims.append(sim)
        self.server.subscribe(self.settings.shared_cob_id, self._receive_pdo)
        self.server.subscribe(self.rs06.target_cob_id, self._receive_pdo)
        self.server.subscribe(SYNC_COB_ID, self._receive_sync)
        self.bus = FourAxisBus(self.settings, self.rs06)
        self.bus.open()
        self.addCleanup(self.bus.close)

    def _reader(self, sim):
        """状态/位置/速度来自协议模拟器，其余读取LocalNode原始对象存储。"""
        def read(index, subindex, od):
            """尊重EDS类型；子索引看门狗也使用对应整数值。"""
            if index == Object.IDENTITY:
                return {0: 3, 1: 1366 if isinstance(sim, SimulatedRS06) else 768,
                        2: 17988, 3: 0}[subindex]
            key = (index, subindex) if subindex else index
            if index == Object.STATUSWORD:
                return sim.read(index)
            return sim.values.get(key) if index >= 0x6000 else None
        return read

    def _writer(self, local, sim):
        """用真实EDS解码SDO，配置值仍保存在LocalNode。"""
        def write(index, subindex, od, data):
            """可注入第四轴映射拒绝，验证前三轴已修改映射也会恢复。"""
            if local.id == 4 and index == 0x1603 and subindex == 1 and self.reject_rs_mapping:
                self.reject_rs_mapping = False
                raise canopen.sdo.SdoAbortedError(0x06040041)
            if index >= 0x6000:
                sim.write(index, od.decode_raw(data), subindex)
        return write

    def _receive_pdo(self, can_id, data, timestamp):
        """只缓存传输类型1的映射；收到命令帧时绝不提前更新运动对象。"""
        for local in self.locals:
            cob_id = int.from_bytes(local.get_data(0x1403, 1), 'little')
            transmission = int.from_bytes(local.get_data(0x1403, 2), 'little')
            if cob_id == can_id and transmission == 1 and local.nmt.state == 'OPERATIONAL':
                self.pending[local.id] = bytes(data)

    def _receive_sync(self, can_id, data, timestamp):
        """以同一接收时间释放两类RPDO，并记录新目标触发时四轴是否同时待处理。"""
        starts = [node for node, frame in self.pending.items()
                  if node != 4 and int.from_bytes(frame, 'little') & CW_NEW_SETPOINT]
        if starts:
            self.releases.append((tuple(sorted(self.pending)), timestamp))
        for local, sim in zip(self.locals, self.sims):
            if local.id not in self.pending:
                continue
            frame = self.pending[local.id]
            mapping = int.from_bytes(local.get_data(0x1603, 1), 'little')
            if mapping == PDO_CONTROLWORD_MAPPING:
                sim.write(Object.CONTROLWORD, int.from_bytes(frame, 'little'))
            elif mapping == TARGET_POSITION_MAPPING:
                sim.write(Object.TARGET_POSITION, int.from_bytes(frame, 'little', signed=True))
        self.pending.clear()

    def test_four_eds_nodes_share_network_and_start_on_same_sync(self):
        """验证身份、100RPM真实编解码、共同SYNC、正反两程和四轴原配置恢复。"""
        self.assertEqual(len(self.bus.verify_identities()), 4)
        self.assertIs(self.bus.axes[0].network, self.bus.rs06_io.network)
        self.assertEqual(self.bus.axes[0].settings.expected_vendor_id, 768)
        self.assertEqual(self.bus.rs06_io.settings.expected_vendor_id, 1366)
        originals = [self.bus._snapshot(io) for io in self.bus.axes + [self.bus.rs06_io]]
        periods = [io.read(0x1006) for io in self.bus.axes + [self.bus.rs06_io]]
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=2), self.rs06)
        report = runner.run()
        self.assertEqual(report['completed_round_trips'], 1)
        self.assertEqual(report['rs06_completed_round_trips'], 1)
        self.assertEqual([nodes for nodes, _ in self.releases], [(1, 2, 3, 4)] * 2)
        self.assertEqual(self.bus.rs06_io.read(Object.PROFILE_VELOCITY), 1000)
        self.assertEqual(self.bus.rs06_io.read(Object.PROFILE_ACCELERATION), 500)
        self.assertEqual([self.bus._snapshot(io) for io in self.bus.axes + [self.bus.rs06_io]], originals)
        self.assertEqual([io.read(0x1006) for io in self.bus.axes + [self.bus.rs06_io]], periods)
        self.assertTrue(all(sim.values[Object.CONTROLWORD] & CW_HALT for sim in self.sims[:3]))
        self.assertEqual(self.sims[3].values[Object.CONTROLWORD], RS06_QUICK_STOP)

    def test_rs06_mapping_rejection_restores_already_changed_kinco(self):
        """第四轴配置失败不得运动，前三轴及第四轴部分映射都恢复原值。"""
        originals = [self.bus._snapshot(io) for io in self.bus.axes + [self.bus.rs06_io]]
        self.reject_rs_mapping = True
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=2), self.rs06)
        with self.assertRaises(TransportError):
            runner.run()
        self.assertFalse(self.releases)
        self.assertEqual([self.bus._snapshot(io) for io in self.bus.axes + [self.bus.rs06_io]], originals)
        self.assertFalse(any(index == Object.CONTROLWORD for sim in self.sims
                             for index, _, _ in sim.writes))

    def test_check_detects_cross_vendor_cob_id_conflict_without_writes(self):
        """RS06其他PDO占用Kinco共享ID时，只读校验立即失败。"""
        self.locals[3].set_data(0x1400, 1, self.settings.shared_cob_id.to_bytes(4, 'little'))
        with self.assertRaisesRegex(TransportError, '占用'):
            self.bus.check_sync_configuration()
        self.assertFalse(any(sim.writes for sim in self.sims))
        self.assertFalse(self.releases)

    def test_read_only_check_never_configures_enables_or_sends_sync(self):
        """四轴--check底层流程只读；可验证身份及PDO但无状态写入。"""
        originals = [self.bus._snapshot(io) for io in self.bus.axes + [self.bus.rs06_io]]
        self.bus.verify_identities()
        self.bus.check_sync_configuration()
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=2), self.rs06)
        self.assertEqual(runner.check()['rs06']['turns'], 100)
        self.assertEqual([self.bus._snapshot(io) for io in self.bus.axes + [self.bus.rs06_io]], originals)
        self.assertFalse(any(sim.writes for sim in self.sims))
        self.assertFalse(self.releases)


if __name__ == '__main__':
    unittest.main()
