"""三轴 PP 往返耐久测试：同步正转/反转，支持全程采样、限时停止及 Ctrl+C。

600 RPM = 10 rps，加/减速各 2 秒时对应 5 rps/s。600 圈的静止起终点
梯形轮廓每程约 62 秒；120 秒观察通常不能完成一次完整往返。
"""

import argparse
import csv
import json
import logging
import time
from collections import deque
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Optional

from .group_controller import ThreeAxisController
from .group_model import GroupSettings, plan_group
from .group_transport import CanopenGroupBus, SimulatedGroupBus
from .model import Motion, integer, positive
from .objects import Object

LOG = logging.getLogger(__name__)


class DurationReached(RuntimeError):
    """达到观察时间；三轴控制器仍须先暂停并确认静止，再恢复配置。"""


@dataclass(frozen=True)
class RunParameters:
    """往返测试参数；圈数为电机轴圈数，速度 rpm，时间秒，None 表示不限时/程数。"""

    turns: float = 600
    velocity_rpm: float = 600
    acceleration_time_s: float = 2
    deceleration_time_s: float = 2
    duration_s: Optional[float] = None
    legs: Optional[int] = None
    sample_interval_s: float = 0.25
    start_direction: str = 'forward'

    def __post_init__(self):
        """校验有限正参数；无效值抛 ValueError，拒绝零时间和无意义的采样周期。"""
        for name in ('turns', 'velocity_rpm', 'acceleration_time_s',
                     'deceleration_time_s', 'sample_interval_s'):
            positive(getattr(self, name), name)
        if self.duration_s is not None:
            positive(self.duration_s, 'duration_s')
        if self.legs is not None:
            integer(self.legs, 1, (1 << 31) - 1, 'legs')
        if self.start_direction not in ('forward', 'reverse'):
            raise ValueError('start_direction 必须是 forward 或 reverse')

    @property
    def acceleration_rps2(self):
        """返回从静止加速至设定速度所需的 rps/s；600 RPM、2s 对应 5。"""
        return self.velocity_rpm / 60 / self.acceleration_time_s

    @property
    def deceleration_rps2(self):
        """返回从设定速度减至静止所需的 rps/s。"""
        return self.velocity_rpm / 60 / self.deceleration_time_s


class ReciprocationRunner:
    """在同一总线上重复同步 PP 单程，锚定起点和远端，避免相对定位误差累积。"""

    def __init__(self, bus, settings: GroupSettings, parameters: RunParameters,
                 clock=time.monotonic, sleep=time.sleep, sample_sink=None,
                 controller_factory=ThreeAxisController):
        """注入三轴总线和可测试时钟；sample_sink 可流式写 CSV，不另开 SDO 连接。"""
        self.bus = bus
        self.settings = settings
        self.parameters = parameters
        self.clock = clock
        self.sleep = sleep
        self.sample_sink = sample_sink
        self.controller_factory = controller_factory
        self.started_at = None
        self.next_sample = 0.0
        self.next_log = 0.0
        self.phase = 'idle'
        self.samples = deque(maxlen=4096)
        self.report = dict(parameters=asdict(parameters), completed_legs=0,
                           completed_full_legs=0, completed_round_trips=0,
                           sample_count=0, recent_legs=[],
                           stop_reason=None, success=False)

    def _check_duration(self):
        """观察时间到达时抛 DurationReached，由控制器正常执行全组减速暂停。"""
        if (self.started_at is not None and self.parameters.duration_s is not None
                and self.clock() - self.started_at >= self.parameters.duration_s):
            raise DurationReached('已达到观察时间，暂停全部轴')

    def _poll(self):
        """在主控制线程检查截止时间并采样；读取失败会进入全组故障暂停路径。"""
        self._check_duration()
        elapsed = self.clock() - self.started_at
        if elapsed < self.next_sample:
            return
        positions, velocities, statuses = [], [], []
        for axis, settings in self._feedback_axes():
            positions.append(axis.read(Object.ACTUAL_POSITION))
            raw = axis.read(Object.ACTUAL_VELOCITY)
            velocities.append(settings.feedback_velocity_rpm(raw))
            statuses.append(axis.read(Object.STATUSWORD))
        row = dict(elapsed_s=elapsed, phase=self.phase, positions_inc=positions,
                   velocities_rpm=velocities, statuswords=statuses)
        self.samples.append(row)
        self.report['sample_count'] += 1
        if self.sample_sink is not None:
            self.sample_sink(row)
        self.next_sample = elapsed + self.parameters.sample_interval_s
        if elapsed >= self.next_log:
            LOG.info('观察 %.2fs，方向=%s，速度反馈 rpm=%s，位置 inc=%s', elapsed, self.phase,
                     ', '.join(f'{v:+.2f}' for v in velocities), positions)
            self.next_log = elapsed + 5

    def _feedback_axes(self):
        """返回反馈 IO 和设备单位配置；伴随轴运行器可追加不同厂家的反馈。"""
        return list(zip(self.bus.axes, (axis.settings for axis in self.settings.axes)))

    def _leg_settings(self, endpoints, starts=None):
        """以固定端点和当前反馈计算相对指令，不自动扩大用户配置的软件运动范围。"""
        axes = []
        if starts is None:
            starts = tuple(io.read(Object.ACTUAL_POSITION) for io in self.bus.axes)
        for config, current, target in zip(self.settings.axes, starts, endpoints):
            motion = Motion(target - current, self.parameters.velocity_rpm,
                            self.parameters.acceleration_rps2,
                            self.parameters.deceleration_rps2, relative=True)
            motion.validate(config.settings, current)
            axes.append(replace(config, motion=motion))
        return replace(self.settings, axes=tuple(axes), duration_s=None)

    def check(self):
        """只读校验当前位置、固定起点、两端及轮廓，返回计划；不使能、不写 PDO。

        配置 origin_position_inc 时重启复用原端点；全部省略时兼容旧版的
        本次启动点锚定方式。首次从区间中途恢复只算恢复段，不冒充600圈完整单程。
        若已在所选方向的端点，则从相反方向开始，避免零位移空计数。
        """
        starts = tuple(io.read(Object.ACTUAL_POSITION) for io in self.bus.axes)
        fixed = all(axis.origin_position_inc is not None for axis in self.settings.axes)
        origin = tuple(axis.origin_position_inc if fixed else start
                       for axis, start in zip(self.settings.axes, starts))
        distant = []
        for config, start in zip(self.settings.axes, origin):
            raw_distance = self.parameters.turns * config.settings.encoder_counts_per_rev
            distance = round(raw_distance)
            if distance <= 0 or abs(raw_distance - distance) > 1e-6:
                raise ValueError('圈数必须能表示为各轴整数 inc')
            distant.append(start + distance)
        plans = []
        for endpoints in (distant, origin):
            try:
                leg = self._leg_settings(endpoints, starts)
                # 相对指令以实际当前位置为基准，不能错误地再叠加到固定起点。
                plans.append(plan_group(leg, starts))
            except ValueError as exc:
                details = '; '.join(
                    f'{a.name}/节点{a.settings.node_id}: 当前={current}, 起点={low}, '
                    f'远端={high}, 范围=[{a.settings.min_position_inc},{a.settings.max_position_inc}]'
                    for a, current, low, high in zip(self.settings.axes, starts, origin, distant))
                raise ValueError(f'往返范围/参数校验失败：{exc}；{details}。未发送运动指令') from exc
        forward = self.parameters.start_direction == 'forward'
        selected = distant if forward else origin
        if all(abs(current - target) <= axis.settings.position_tolerance_inc
               for axis, current, target in zip(self.settings.axes, starts, selected)):
            forward = not forward
        initial_endpoint = origin if forward else distant
        partial = any(abs(current - target) > axis.settings.position_tolerance_inc
                      for axis, current, target in zip(self.settings.axes, starts, initial_endpoint))
        return dict(start_positions_inc=starts, origin_positions_inc=origin,
                    far_positions_inc=distant, fixed_origins=fixed,
                    effective_start_direction='forward' if forward else 'reverse',
                    partial_first_leg=partial,
                    first_target_positions_inc=distant if forward else origin,
                    first_leg_estimated_s=plans[0 if forward else 1].duration_s)

    def run(self):
        """运行至限时/程数或中断，返回报告；通信/故障/停机失败仍抛异常。

        默认无限往返。每程结束均确认停稳后才换向。限时可在途中发生，
        暂停按配置减速度执行，因此物理停止和配置恢复可能略晚于观察截止时间。
        内存仅保留最近 4096 个样本和 100 程摘要，完整采样可经 sink 写 CSV。
        """
        try:
            checked = self.check()
            self.report.update(checked)
            origin, distant = checked['origin_positions_inc'], checked['far_positions_inc']
            first_forward = checked['effective_start_direction'] == 'forward'
            self.started_at = self.clock()
            self.phase = checked['effective_start_direction']
            while True:
                self._check_duration()
                even_leg = self.report['completed_legs'] % 2 == 0
                forward = first_forward if even_leg else not first_forward
                self.phase = 'forward' if forward else 'reverse'
                endpoints = distant if forward else origin
                leg = self._leg_settings(endpoints)
                controller = self.controller_factory(self.bus, leg, self.clock, self.sleep,
                                                     poll_hook=self._poll)
                LOG.info('开始第 %d 程：%s，目标端点=%s，速度上限=%.2f rpm，加/减速=%.3f/%.3fs',
                         self.report['completed_legs'] + 1, self.phase, endpoints,
                         self.parameters.velocity_rpm, self.parameters.acceleration_time_s,
                         self.parameters.deceleration_time_s)
                result = controller.run()
                count = self.report['completed_legs'] + 1
                full_stroke = not (count == 1 and checked['partial_first_leg'])
                full_count = self.report['completed_full_legs'] + int(full_stroke)
                self.report.update(completed_legs=count, completed_full_legs=full_count,
                                   completed_round_trips=full_count // 2)
                self.report['recent_legs'].append(dict(number=count, phase=self.phase,
                    full_stroke=full_stroke, positions_inc=result.positions,
                    observed_arrival_spread_s=result.observed_arrival_spread_s))
                self.report['recent_legs'] = self.report['recent_legs'][-100:]
                if self.parameters.legs is not None and count >= self.parameters.legs:
                    self.report['stop_reason'] = 'leg_limit'
                    break
            self.report['success'] = True
        except DurationReached:
            self.report.update(stop_reason='duration', success=True)
            LOG.info('观察时间结束，全部轴停止与配置清理已完成')
        except KeyboardInterrupt:
            self.report.update(stop_reason='interrupt', success=True)
            LOG.info('收到 Ctrl+C，全部轴停止与配置清理已完成')
        except BaseException as exc:
            self.report.update(stop_reason='error', error=str(exc))
            raise
        finally:
            self.report['elapsed_s'] = self.clock() - self.started_at if self.started_at is not None else 0
            self.report['last_phase'] = self.phase
            self.report['samples'] = list(self.samples)
        # 这里的只读复查在全组停止/映射恢复后进行，异常不能误报成功。
        try:
            self.report['final_states'] = [dict(node_id=settings.node_id,
                position_inc=io.read(Object.ACTUAL_POSITION), controlword=io.read(Object.CONTROLWORD),
                statusword=io.read(Object.STATUSWORD), mode=io.read(Object.MODE_DISPLAY),
                velocity_raw=io.read(Object.ACTUAL_VELOCITY))
                for io, settings in self._feedback_axes()]
            for row, (_, settings) in zip(self.report['final_states'], self._feedback_axes()):
                row['velocity_rpm'] = settings.feedback_velocity_rpm(row['velocity_raw'])
        except Exception as exc:
            self.report.update(success=False, error=f'停止后状态读取失败：{exc}')
            raise
        return self.report


def main(argv=None):
    """命令行入口；默认模拟且无限往返，--execute 才驱动实机，--duration 限时。

    --check只读校验端点；--start-direction reverse先继续回到原起点。
    出错返回 1，Ctrl+C 正常停止返回 130，限时或指定程数完成返回 0。
    --output 保存 JSON 摘要，--csv 流式记录完整反馈，均由用户显式指定路径。
    """
    parser = argparse.ArgumentParser(description='三轴、四轴或七轴 PP 往返；默认模拟，Ctrl+C 停止')
    parser.add_argument('--config', default='config/reciprocate.example.json')
    companions = parser.add_mutually_exclusive_group()
    companions.add_argument('--rs06-config', help='附加RS06独立配置，启用四轴共同SYNC往返')
    companions.add_argument('--robstride-config', help='四台RobStride独立端点配置，启用七轴共同SYNC往返')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='执行已核对配置的实机运动')
    mode.add_argument('--check', action='store_true', help='只连接实机读取并校验往返端点，不触发运动')
    parser.add_argument('--turns', type=float, default=600, help='单程圈数，默认600')
    parser.add_argument('--velocity', type=float, default=600, help='轮廓速度rpm，默认600')
    parser.add_argument('--accel-time', type=float, default=2, help='0到设定速度的加速秒数')
    parser.add_argument('--decel-time', type=float, default=2, help='设定速度到0的减速秒数')
    parser.add_argument('--duration', type=float, help='观察秒数，到时减速暂停；省略则不限时')
    parser.add_argument('--legs', type=int, help='最多到位段数（含恢复段）；无恢复段时2段为完整往返')
    parser.add_argument('--sample-interval', type=float, default=0.25, help='同线程采样间隔秒数')
    parser.add_argument('--start-direction', choices=('forward', 'reverse'), default='forward',
                        help='重启后首先前往正向远端或原起点，默认forward')
    parser.add_argument('--output', help='JSON结果文件路径')
    parser.add_argument('--csv', help='完整反馈CSV文件路径')
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    bus, runner, csv_file = None, None, None
    code = 1
    try:
        parameters = RunParameters(args.turns, args.velocity, args.accel_time, args.decel_time,
                                   args.duration, args.legs, args.sample_interval, args.start_direction)
        settings = GroupSettings.load(args.config)
        rs06, profiles = None, ()
        if args.rs06_config or args.robstride_config:
            from .rs06 import RS06Settings
            from .four_axis_reciprocate import (FourAxisBus, FourAxisReciprocationRunner,
                                                SimulatedFourAxisBus, validate_topology)
            if args.robstride_config:
                from .robstride import load_seven_axis_profiles
                from .seven_axis_reciprocate import (SevenAxisBus, SevenAxisReciprocationRunner,
                                                     SimulatedSevenAxisBus)
                profiles = load_seven_axis_profiles(args.robstride_config)
                rs06 = profiles[0]
            else:
                rs06 = RS06Settings.load(args.rs06_config)
                profiles = (rs06,)
            validate_topology(settings, rs06, profiles[1:])
            if not args.execute and not args.check:
                # 模拟器坐标由本地初始化确定，仅在纯模拟时建立绝对坐标确认。
                # 实机执行和只读检查均保留用户配置的确认标志。
                settings = replace(settings, axes=tuple(replace(axis,
                    settings=replace(axis.settings, absolute_reference_verified=True))
                    for axis in settings.axes))
            if args.execute:
                for profile in profiles:
                    if not (profile.hardware_verified and profile.absolute_reference_verified
                            and profile.synchronous_pdo_verified):
                        raise ValueError(f"{getattr(profile, 'model', 'RS06')}/节点{profile.node_id} 实机运行前须核对 "
                                         'hardware_verified、absolute_reference_verified、synchronous_pdo_verified')
            if args.execute and not all(axis.settings.absolute_reference_verified for axis in settings.axes):
                raise ValueError('混合多轴固定绝对端点运行前须逐轴确认Kinco absolute_reference_verified')
        if args.execute and (not settings.pdo_configuration_verified
                or not all(a.settings.hardware_verified for a in settings.axes)):
            raise ValueError('实机运行前必须核对所有 hardware_verified 和 pdo_configuration_verified')
        writer = None
        if args.csv:
            csv_file = Path(args.csv).open('x', newline='', encoding='utf-8')
            writer = csv.writer(csv_file)
            nodes = [a.settings.node_id for a in settings.axes]
            nodes.extend(profile.node_id for profile in profiles)
            writer.writerow(['elapsed_s', 'phase']
                            + [f'node{node}_rpm' for node in nodes]
                            + [f'node{node}_inc' for node in nodes]
                            + [f'node{node}_status' for node in nodes])
        def sink(row):
            """将同一控制线程产生的反馈流式写出，磁盘失败同样触发全组暂停。"""
            if writer is not None:
                writer.writerow([row['elapsed_s'], row['phase']] + row['velocities_rpm']
                                + row['positions_inc'] + row['statuswords'])
                csv_file.flush()
        if args.execute or args.check:
            if args.robstride_config:
                bus = SevenAxisBus(settings, profiles)
            else:
                bus = CanopenGroupBus(settings) if rs06 is None else FourAxisBus(settings, rs06)
            bus.open()
            LOG.info('全部轴身份：%s', bus.verify_identities())
            if rs06 is not None:
                bus.check_sync_configuration()
        else:
            LOG.info('模拟往返，不连接CAN或真实电机；模拟器不模拟机械运动时间')
            if args.robstride_config:
                bus = SimulatedSevenAxisBus(tuple(axis.origin_position_inc for axis in settings.axes),
                                             tuple(profile.origin_position_inc for profile in profiles))
            else:
                bus = SimulatedGroupBus() if rs06 is None else SimulatedFourAxisBus()
        if args.robstride_config:
            runner = SevenAxisReciprocationRunner(bus, settings, parameters, profiles, sample_sink=sink)
        else:
            runner = (ReciprocationRunner(bus, settings, parameters, sample_sink=sink) if rs06 is None
                      else FourAxisReciprocationRunner(bus, settings, parameters, rs06, sample_sink=sink))
        if args.check:
            checked = runner.check()
            runner.report.update(checked, check_only=True, success=True, stop_reason='check')
            LOG.info('只读端点校验通过：当前位置=%s，固定起点=%s，远端=%s，首次方向=%s，首程为恢复段=%s',
                     checked['start_positions_inc'], checked['origin_positions_inc'],
                     checked['far_positions_inc'], checked['effective_start_direction'], checked['partial_first_leg'])
            if profiles:
                LOG.info('RobStride逐轴端点：%s', checked['robstride_axes'])
            LOG.info('未发送NMT启动、PDO配置、使能或运动指令')
            code = 0
        else:
            report = runner.run()
            LOG.info('结束：原因=%s，到位段=%d，完整单程=%d，完整往返=%d，经过=%.3fs，最后方向=%s',
                     report['stop_reason'], report['completed_legs'], report['completed_full_legs'],
                     report['completed_round_trips'], report['elapsed_s'], report['last_phase'])
            code = 130 if report['stop_reason'] == 'interrupt' else 0
    except Exception as exc:
        LOG.error('往返测试失败：%s', exc)
    finally:
        if csv_file is not None:
            csv_file.close()
        if runner is not None and args.output:
            try:
                with Path(args.output).open('x', encoding='utf-8') as stream:
                    json.dump(runner.report, stream, ensure_ascii=False, indent=2)
                    stream.write('\n')
            except Exception as exc:
                LOG.error('保存报告失败：%s', exc)
                code = 1
        if (args.execute or args.check) and bus is not None:
            try:
                bus.close()
            except Exception as exc:
                LOG.error('关闭总线失败：%s', exc)
                code = 1
    return code


if __name__ == '__main__':
    raise SystemExit(main())
