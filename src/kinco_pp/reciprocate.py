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

    def __post_init__(self):
        """校验有限正参数；无效值抛 ValueError，拒绝零时间和无意义的采样周期。"""
        for name in ('turns', 'velocity_rpm', 'acceleration_time_s',
                     'deceleration_time_s', 'sample_interval_s'):
            positive(getattr(self, name), name)
        if self.duration_s is not None:
            positive(self.duration_s, 'duration_s')
        if self.legs is not None:
            integer(self.legs, 1, (1 << 31) - 1, 'legs')

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
                 clock=time.monotonic, sleep=time.sleep, sample_sink=None):
        """注入三轴总线和可测试时钟；sample_sink 可流式写 CSV，不另开 SDO 连接。"""
        self.bus = bus
        self.settings = settings
        self.parameters = parameters
        self.clock = clock
        self.sleep = sleep
        self.sample_sink = sample_sink
        self.started_at = None
        self.next_sample = 0.0
        self.next_log = 0.0
        self.phase = 'idle'
        self.samples = deque(maxlen=4096)
        self.report = dict(parameters=asdict(parameters), completed_legs=0,
                           completed_round_trips=0, sample_count=0, recent_legs=[],
                           stop_reason=None, success=False)

    def _check_duration(self):
        """观察时间到达时抛 DurationReached，由控制器正常执行全组减速暂停。"""
        if (self.started_at is not None and self.parameters.duration_s is not None
                and self.clock() - self.started_at >= self.parameters.duration_s):
            raise DurationReached('已达到观察时间，暂停三轴')

    def _poll(self):
        """在主控制线程检查截止时间并采样；读取失败会进入全组故障暂停路径。"""
        self._check_duration()
        elapsed = self.clock() - self.started_at
        if elapsed < self.next_sample:
            return
        positions, velocities, statuses = [], [], []
        for axis, config in zip(self.bus.axes, self.settings.axes):
            positions.append(axis.read(Object.ACTUAL_POSITION))
            raw = axis.read(Object.ACTUAL_VELOCITY)
            velocities.append(raw * 1875 / (512 * config.settings.encoder_counts_per_rev))
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

    def _leg_settings(self, endpoints):
        """以固定端点和当前反馈计算相对指令，不自动扩大用户配置的软件运动范围。"""
        axes = []
        for config, io, target in zip(self.settings.axes, self.bus.axes, endpoints):
            current = io.read(Object.ACTUAL_POSITION)
            motion = Motion(target - current, self.parameters.velocity_rpm,
                            self.parameters.acceleration_rps2,
                            self.parameters.deceleration_rps2, relative=True)
            motion.validate(config.settings, current)
            axes.append(replace(config, motion=motion))
        return replace(self.settings, axes=tuple(axes), duration_s=None)

    def run(self):
        """运行至限时/程数或中断，返回报告；通信/故障/停机失败仍抛异常。

        默认无限往返。每程结束均确认停稳后才换向。限时可在途中发生，
        暂停按配置减速度执行，因此物理停止和配置恢复可能略晚于观察截止时间。
        内存仅保留最近 4096 个样本和 100 程摘要，完整采样可经 sink 写 CSV。
        """
        try:
            origin = tuple(axis.read(Object.ACTUAL_POSITION) for axis in self.bus.axes)
            distant = []
            for config, start in zip(self.settings.axes, origin):
                raw_distance = self.parameters.turns * config.settings.encoder_counts_per_rev
                distance = round(raw_distance)
                if distance <= 0 or abs(raw_distance - distance) > 1e-6:
                    raise ValueError('圈数必须能表示为各轴整数 inc')
                distant.append(start + distance)
            self.report.update(origin_positions_inc=origin, far_positions_inc=distant)
            # 正反两端在任何轴使能前都校验，避免去程可行而回程越界。
            for endpoints in (distant, origin):
                leg = self._leg_settings(endpoints)
                plan_group(leg, origin)
            self.started_at = self.clock()
            self.phase = 'forward'
            while True:
                self._check_duration()
                forward = self.report['completed_legs'] % 2 == 0
                self.phase = 'forward' if forward else 'reverse'
                endpoints = distant if forward else origin
                leg = self._leg_settings(endpoints)
                controller = ThreeAxisController(self.bus, leg, self.clock, self.sleep,
                                                 poll_hook=self._poll)
                LOG.info('开始第 %d 程：%s，目标端点=%s，速度上限=%.2f rpm，加/减速=%.3f/%.3fs',
                         self.report['completed_legs'] + 1, self.phase, endpoints,
                         self.parameters.velocity_rpm, self.parameters.acceleration_time_s,
                         self.parameters.deceleration_time_s)
                result = controller.run()
                count = self.report['completed_legs'] + 1
                self.report.update(completed_legs=count, completed_round_trips=count // 2)
                self.report['recent_legs'].append(dict(number=count, phase=self.phase,
                    positions_inc=result.positions, observed_arrival_spread_s=result.observed_arrival_spread_s))
                self.report['recent_legs'] = self.report['recent_legs'][-100:]
                if self.parameters.legs is not None and count >= self.parameters.legs:
                    self.report['stop_reason'] = 'leg_limit'
                    break
            self.report['success'] = True
        except DurationReached:
            self.report.update(stop_reason='duration', success=True)
            LOG.info('观察时间结束，三轴暂停与配置清理已完成')
        except KeyboardInterrupt:
            self.report.update(stop_reason='interrupt', success=True)
            LOG.info('收到 Ctrl+C，三轴暂停与配置清理已完成')
        except BaseException as exc:
            self.report.update(stop_reason='error', error=str(exc))
            raise
        finally:
            self.report['elapsed_s'] = self.clock() - self.started_at if self.started_at is not None else 0
            self.report['last_phase'] = self.phase
            self.report['samples'] = list(self.samples)
        # 这里的只读复查在全组停止/映射恢复后进行，异常不能误报成功。
        try:
            self.report['final_states'] = [dict(node_id=config.settings.node_id,
                position_inc=io.read(Object.ACTUAL_POSITION), controlword=io.read(Object.CONTROLWORD),
                statusword=io.read(Object.STATUSWORD), mode=io.read(Object.MODE_DISPLAY),
                velocity_raw=io.read(Object.ACTUAL_VELOCITY))
                for config, io in zip(self.settings.axes, self.bus.axes)]
            for row, config in zip(self.report['final_states'], self.settings.axes):
                row['velocity_rpm'] = row['velocity_raw'] * 1875 / (512 * config.settings.encoder_counts_per_rev)
        except Exception as exc:
            self.report.update(success=False, error=f'停止后状态读取失败：{exc}')
            raise
        return self.report


def main(argv=None):
    """命令行入口；默认模拟且无限往返，--execute 才驱动实机，--duration 限时。

    出错返回 1，Ctrl+C 正常停止返回 130，限时或指定程数完成返回 0。
    --output 保存 JSON 摘要，--csv 流式记录完整反馈，均由用户显式指定路径。
    """
    parser = argparse.ArgumentParser(description='三轴 PP 正反往返测试；默认模拟，Ctrl+C 暂停')
    parser.add_argument('--config', default='config/reciprocate.example.json')
    parser.add_argument('--execute', action='store_true', help='执行已核对配置的实机运动')
    parser.add_argument('--turns', type=float, default=600, help='单程圈数，默认600')
    parser.add_argument('--velocity', type=float, default=600, help='轮廓速度rpm，默认600')
    parser.add_argument('--accel-time', type=float, default=2, help='0到设定速度的加速秒数')
    parser.add_argument('--decel-time', type=float, default=2, help='设定速度到0的减速秒数')
    parser.add_argument('--duration', type=float, help='观察秒数，到时减速暂停；省略则不限时')
    parser.add_argument('--legs', type=int, help='最多单程数；2程为一次完整往返')
    parser.add_argument('--sample-interval', type=float, default=0.25, help='同线程采样间隔秒数')
    parser.add_argument('--output', help='JSON结果文件路径')
    parser.add_argument('--csv', help='完整反馈CSV文件路径')
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    bus, runner, csv_file = None, None, None
    code = 1
    try:
        parameters = RunParameters(args.turns, args.velocity, args.accel_time, args.decel_time,
                                   args.duration, args.legs, args.sample_interval)
        settings = GroupSettings.load(args.config)
        if args.execute and (not settings.pdo_configuration_verified
                or not all(a.settings.hardware_verified for a in settings.axes)):
            raise ValueError('实机运行前必须核对所有 hardware_verified 和 pdo_configuration_verified')
        writer = None
        if args.csv:
            csv_file = Path(args.csv).open('x', newline='', encoding='utf-8')
            writer = csv.writer(csv_file)
            nodes = [a.settings.node_id for a in settings.axes]
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
        if args.execute:
            bus = CanopenGroupBus(settings)
            bus.open()
            LOG.info('三轴身份：%s', bus.verify_identities())
        else:
            LOG.info('模拟往返，不连接CAN或真实电机；模拟器不模拟机械运动时间')
            bus = SimulatedGroupBus()
        runner = ReciprocationRunner(bus, settings, parameters, sample_sink=sink)
        report = runner.run()
        LOG.info('结束：原因=%s，完成单程=%d，完整往返=%d，经过=%.3fs，最后方向=%s',
                 report['stop_reason'], report['completed_legs'], report['completed_round_trips'],
                 report['elapsed_s'], report['last_phase'])
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
        if args.execute and bus is not None:
            try:
                bus.close()
            except Exception as exc:
                LOG.error('关闭总线失败：%s', exc)
                code = 1
    return code


if __name__ == '__main__':
    raise SystemExit(main())
