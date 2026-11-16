"""四轴真实 EDS/SDO/同步 RPDO 协议回归；仅连接 python-can virtual，不连接 can0。"""

import unittest
import uuid
from contextlib import ExitStack
from dataclasses import replace
from unittest.mock import patch

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
                                           sdo_timeout_s=0.2, absolute_reference_verified=True)) for axis in source.axes))
        self.rs06 = replace(RS06Settings.load('config/rs06.example.json'),
                            interface='virtual', channel=channel, synchronous_pdo_verified=True)
        self.server = canopen.Network()
        self.server.connect(interface='virtual', channel=channel)
        self.addCleanup(self.server.disconnect)
        self.locals, self.sims, self.pending, self.releases = [], [], {}, []
        self.reject_rs_mapping = False
        self.pending_rs_controlword = None
        self.rs_sync_armed = False
        self.legacy_control_collisions = 0
        self.drop_feedback_node = None
        configs = [axis.settings for axis in self.settings.axes] + [self.rs06]
        for config in configs:
            local = canopen.LocalNode(config.node_id, config.eds)
            self.server.add_node(local)
            sim = SimulatedRS06() if config.node_id == self.rs06.node_id else SimulatedDrive()
            local.set_data(0x1603, 0, (1).to_bytes(1, 'little'))
            local.set_data(0x1603, 1, (PDO_CONTROLWORD_MAPPING).to_bytes(4, 'little'))
            if config.node_id != self.rs06.node_id:
                local.set_data(0x1A00, 0, (3).to_bytes(1, 'little'))
                for sub, mapping in enumerate(((int(Object.STATUSWORD)<<16)|16,
                        (config.actual_position_index<<16)|32, (int(Object.MODE_DISPLAY)<<16)|8), 1):
                    local.set_data(0x1A00, sub, mapping.to_bytes(4, 'little'))
                local.set_data(0x1800, 1, (0x180+config.node_id).to_bytes(4, 'little'))
                local.set_data(0x1800, 2, (1).to_bytes(1, 'little'))
                local.set_data(0x1801, 1, (0x280+config.node_id).to_bytes(4, 'little'))
                local.set_data(0x1801, 2, (254).to_bytes(1, 'little'))
                local.set_data(0x1801, 3, (0).to_bytes(2, 'little'))
                local.set_data(0x1A01, 0, (0).to_bytes(1, 'little'))
                # 复现实机遗留的同步RPDO1，检查运行时会临时隔离其他控制通道。
                local.set_data(0x1600, 0, (3).to_bytes(1, 'little'))
                for sub, mapping in enumerate((TARGET_POSITION_MAPPING,
                        (int(Object.MODE)<<16)|8, PDO_CONTROLWORD_MAPPING), 1):
                    local.set_data(0x1600, sub, mapping.to_bytes(4, 'little'))
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
            if local.id != 4 and index == 0x1006 and od.decode_raw(data) == 0:
                # 实机Kinco不接受0周期；不能在运动前强制改为0。
                raise canopen.sdo.SdoAbortedError(0x06090030)
            if index >= 0x6000:
                if local.id == 4 and index == Object.CONTROLWORD:
                    self.rs_sync_armed = False
                    transmission = int.from_bytes(local.get_data(0x1403, 2), 'little')
                    if transmission == 1:
                        # 复现实机：同步PDO门控下，即使在预操作态，SDO控制字读回已更新，
                        # 实际状态直到SYNC才变化，旧版准备流程会在此超时。
                        self.pending_rs_controlword = od.decode_raw(data)
                        sim.values[index] = self.pending_rs_controlword
                        sim.writes.append((index, self.pending_rs_controlword, subindex))
                        return
                    self.pending_rs_controlword = None
                sim.write(index, od.decode_raw(data), subindex)
        return write

    def _receive_pdo(self, can_id, data, timestamp):
        """只缓存传输类型1的映射；收到命令帧时绝不提前更新运动对象。"""
        for local in self.locals:
            cob_id = int.from_bytes(local.get_data(0x1403, 1), 'little')
            transmission = int.from_bytes(local.get_data(0x1403, 2), 'little')
            if cob_id == can_id and transmission == 1 and local.nmt.state == 'OPERATIONAL':
                if local.id == 4 and not self.rs_sync_armed:
                    # 实机进入OP后的首条SYNC先初始化接收通道，之前的PDO被忽略。
                    continue
                self.pending[local.id] = bytes(data)

    def _receive_sync(self, can_id, data, timestamp):
        """以同一接收时间释放两类RPDO，并记录新目标触发时四轴是否同时待处理。"""
        starts = [node for node, frame in self.pending.items()
                  if node != 4 and int.from_bytes(frame, 'little') & CW_NEW_SETPOINT]
        if starts:
            self.releases.append((tuple(sorted(self.pending)), timestamp))
            for local in self.locals[:3]:
                if not int.from_bytes(local.get_data(0x1400, 1), 'little') & PDO_DISABLED:
                    self.legacy_control_collisions += 1
        if self.locals[3].nmt.state == 'OPERATIONAL':
            self.rs_sync_armed = True
        if self.pending_rs_controlword is not None and self.locals[3].nmt.state == 'OPERATIONAL':
            self.sims[3].write(Object.CONTROLWORD, self.pending_rs_controlword)
            self.pending_rs_controlword = None
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
        for local, sim in zip(self.locals[:3], self.sims[:3]):
            if local.nmt.state != 'OPERATIONAL':
                continue
            status = sim.read(Object.STATUSWORD)
            if local.id == self.drop_feedback_node:
                continue
            payload = (status.to_bytes(2, 'little')
                       + sim.values[Object.ACTUAL_POSITION].to_bytes(4, 'little', signed=True)
                       + sim.values[Object.MODE_DISPLAY].to_bytes(1, 'little', signed=True))
            state_cob = int.from_bytes(local.get_data(0x1800, 1), 'little')
            if not state_cob & PDO_DISABLED:
                self.server.send_message(state_cob & 0x7FF, payload)
            speed_cob = int.from_bytes(local.get_data(0x1801, 1), 'little')
            speed_count = int.from_bytes(local.get_data(0x1A01, 0), 'little')
            if not speed_cob & PDO_DISABLED and speed_count == 1:
                self.server.send_message(speed_cob & 0x7FF,
                    sim.values[Object.ACTUAL_VELOCITY].to_bytes(4, 'little', signed=True))

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
        self.assertEqual(self.legacy_control_collisions, 0)
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

    def test_rs06_initial_zero_sync_cob_id_is_temporary_and_restored(self):
        """RS06实机初始0x1005=0；只读检查接受，运行时配置0x80，结束恢复0。"""
        self.locals[3].set_data(0x1005, 0, (0).to_bytes(4, 'little'))
        self.bus.check_sync_configuration()
        self.assertEqual(self.bus.rs06_io.read(0x1005), 0)
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=2), self.rs06)
        report = runner.run()
        self.assertEqual(report['rs06_completed_round_trips'], 1)
        self.assertEqual(self.bus.rs06_io.read(0x1005), 0)
        self.assertEqual([nodes for nodes, _ in self.releases], [(1, 2, 3, 4)] * 2)

    def test_cyclic_sync_is_active_before_first_post_trigger_poll(self):
        """复现长行程首轮反馈超时前置条件：新目标触发后、等待应答前就要维持SYNC。"""
        broadcast = self.bus.broadcast
        checked = []
        def verify(controlword):
            """每次共同上升沿返回时检查周期任务，而非等待运动结束才检查。"""
            broadcast(controlword)
            if controlword & CW_NEW_SETPOINT:
                checked.append(True)
                self.assertTrue(self.bus._cyclic_sync_active)
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=1), self.rs06)
        with patch.object(self.bus, 'broadcast', side_effect=verify):
            runner.run()
        self.assertEqual(checked, [True])
        self.assertFalse(self.bus._cyclic_sync_active)

    def test_kinco_6063_feedback_selection_does_not_change_rs06_6064(self):
        """真实EDS的SDO编码验证：Kinco按手册选6063，RS06仍读6064，避免跨型号硬套。"""
        io = self.bus.axes[0]
        io.settings = replace(io.settings, actual_position_index=0x6063)
        self.sims[0].values[0x6063] = 12345
        self.sims[0].values[Object.ACTUAL_POSITION] = 54321
        self.sims[3].values[0x6063] = 11111
        self.sims[3].values[Object.ACTUAL_POSITION] = 67890
        self.assertEqual(io.read(Object.ACTUAL_POSITION), 12345)
        self.assertEqual(self.bus.rs06_io.read(Object.ACTUAL_POSITION), 67890)

    def test_motion_feedback_does_not_poll_kinco_sdo_objects(self):
        """运动期禁止Kinco反馈SDO访问，仍可凭TPDO1/2完成同帧触发和定位。"""
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=1), self.rs06)
        def guarded(io):
            """保留非运动期SDO；模拟实机同步TPDO发送时反馈SDO不能可靠应答。"""
            variable = io._variable
            def access(index, subindex):
                if self.bus._cyclic_sync_active and index in (Object.STATUSWORD, Object.MODE_DISPLAY,
                        Object.ACTUAL_POSITION, Object.ACTUAL_VELOCITY):
                    raise RuntimeError('运动期反馈SDO被测试禁用')
                return variable(index, subindex)
            return access
        with ExitStack() as stack:
            for io in self.bus.axes:
                stack.enter_context(patch.object(io, '_variable', side_effect=guarded(io)))
            report = runner.run()
        self.assertTrue(report['success'])
        self.assertEqual(report['completed_legs'], 1)
        self.assertFalse(self.bus.feedback.changed)

    def test_missing_tpdo_feedback_stops_every_axis(self):
        """节点1不再上报TPDO时按通信超时停止全部轴，不读旧缓存冒充正常运动。"""
        self.drop_feedback_node = 1
        runner = FourAxisReciprocationRunner(self.bus, self.settings, RunParameters(legs=1), self.rs06)
        with self.assertRaisesRegex(Exception, 'TPDO'):
            runner.run()
        self.assertTrue(all(sim.values[Object.CONTROLWORD] & CW_HALT for sim in self.sims[:3]))
        self.assertEqual(self.sims[3].values[Object.CONTROLWORD], RS06_QUICK_STOP)
        self.assertFalse(self.bus._cyclic_sync_active)


if __name__ == '__main__':
    unittest.main()
