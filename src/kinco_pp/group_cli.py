"""三轴命令入口；默认使用模拟器，--execute 才会配置 RPDO 并驱动三轴。"""

import argparse
import logging
from typing import Optional, Sequence

from .group_controller import ThreeAxisController
from .group_model import GroupSettings, plan_group
from .group_transport import CanopenGroupBus, SimulatedGroupBus
from .objects import Object

LOG = logging.getLogger(__name__)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """执行配置中的一次三轴运动；成功返回 0，异常返回 1，中断返回 130。

    --inspect 只读三轴身份和位置，不配置 PDO、不启动 NMT、不使能。
    实机配置必须逐轴 hardware_verified=true，且已确认 RPDO 通道可使用。
    """
    parser = argparse.ArgumentParser(description="三轴 PP 同帧启动与共同时间规划；默认模拟")
    parser.add_argument("--config", default="config/three_axis.example.json", help="三轴 JSON 配置")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="配置共享 RPDO，执行真实三轴定位")
    mode.add_argument("--inspect", action="store_true", help="只读三轴身份、状态和位置")
    parser.add_argument("--verbose", action="store_true", help="打印 SDO 通信详情")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")
    bus = None
    result = 0
    try:
        settings = GroupSettings.load(args.config)
        if args.execute:
            if not settings.pdo_configuration_verified:
                raise ValueError("请先确认 RPDO 槽位、共享 COB-ID，并设置 pdo_configuration_verified=true")
            if not all(axis.settings.hardware_verified for axis in settings.axes):
                raise ValueError("实机执行前必须逐轴设置 hardware_verified=true")
        if args.execute or args.inspect:
            bus = CanopenGroupBus(settings)
            bus.open()
            identities = bus.verify_identities()
            for axis, io, identity in zip(settings.axes, bus.axes, identities):
                LOG.info("轴 %s / 节点 %d：身份=%s，状态=0x%04X，位置=%d inc",
                         axis.name, axis.settings.node_id, identity,
                         io.read(Object.STATUSWORD), io.read(Object.ACTUAL_POSITION))
        else:
            LOG.info("三轴模拟：不连接 CAN，不驱动真实电机")
            bus = SimulatedGroupBus()
        if not args.inspect:
            starts = tuple(io.read(Object.ACTUAL_POSITION) for io in bus.axes)
            plan = plan_group(settings, starts)
            for axis_plan in plan.axes:
                LOG.info("轴 %s：目标=%d inc，速度=%.6f rpm，加/减速度=%.6f/%.6f rps/s，预计 %.6fs",
                         axis_plan.axis.name, axis_plan.end_inc, axis_plan.motion.velocity_rpm,
                         axis_plan.motion.acceleration_rps2, axis_plan.motion.deceleration_rps2,
                         axis_plan.estimated_duration_s)
            completed = ThreeAxisController(bus, settings).run()
            LOG.info("%s三轴定位完成：%s；轮询观察到的到达时差 %.6fs",
                     "实机" if args.execute else "模拟", completed.positions,
                     completed.observed_arrival_spread_s)
            LOG.info("三轴流程结束，停止检查与临时配置清理已完成")
    except KeyboardInterrupt:
        LOG.error("运行被中断；三轴运行器已尝试暂停所有已开始使能的轴")
        result = 130
    except Exception as exc:
        LOG.error("三轴运行失败：%s", exc)
        result = 1
    finally:
        if bus is not None and (args.execute or args.inspect):
            try:
                bus.close()
            except Exception as exc:
                LOG.error("关闭共享 CAN 总线失败：%s", exc)
                result = result or 1
    return result


if __name__ == "__main__":
    raise SystemExit(main())
