"""命令行入口；默认使用模拟驱动器，--execute 才访问真实 CANable。"""

import argparse
import logging
from typing import Optional, Sequence

from .controller import ProfilePositionController
from .model import Motion, Settings
from .objects import Object
from .simulation import SimulatedDrive
from .transport import CanopenTransport

LOG = logging.getLogger(__name__)


def parser() -> argparse.ArgumentParser:
    """构造命令行参数并返回解析器；位置单位 inc，速度 rpm，加减速度 rps/s。"""
    result = argparse.ArgumentParser(description="Kinco CANopen PP 单轴定位；默认模拟运行")
    result.add_argument("--config", default="config/servo.example.json", help="JSON 配置路径")
    result.add_argument("--execute", action="store_true", help="连接真实硬件并执行定位")
    result.add_argument("--inspect", action="store_true", help="仅连接并读取身份、状态和模式，不写对象")
    group = result.add_mutually_exclusive_group()
    group.add_argument("--absolute", action="store_true", help="绝对定位；必须已核对坐标原点")
    group.add_argument("--relative", action="store_true", help="相对定位（默认）")
    result.add_argument("--target", type=int, default=1000, help="绝对位置或相对位移 inc，默认 1000")
    result.add_argument("--velocity", type=float, default=1.0, help="梯形速度 rpm")
    result.add_argument("--acceleration", type=float, default=1.0, help="梯形加速度 rps/s")
    result.add_argument("--deceleration", type=float, default=1.0, help="梯形减速度 rps/s")
    result.add_argument("--verbose", action="store_true", help="打印 SDO 读写详情")
    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    """运行一次定位或只读检查；成功返回 0，失败返回 1，中断返回 130。

    正常、故障或 Ctrl+C 路径均在已开始使能后尝试 Halt，随后释放连接。
    暂停失败会报告错误，不能假定真实电机已停止。结束时保持使能锁轴。
    """
    args = parser().parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(levelname)s %(message)s")
    transport = None
    controller = None
    result = 0
    try:
        if args.inspect and args.execute:
            raise ValueError("--inspect 与 --execute 不能同时使用")
        settings = Settings.load(args.config)
        motion = Motion(args.target, args.velocity, args.acceleration,
                        args.deceleration, relative=not args.absolute)
        if args.execute and not settings.hardware_verified:
            raise ValueError("请先核对设备、编码器分辨率、运动范围与停止方式，再设置 hardware_verified=true")
        if args.execute or args.inspect:
            transport = CanopenTransport(settings)
            transport.open()
            LOG.info("读取设备身份：%s", transport.verify_identity())
            if args.inspect:
                LOG.info("只读检查：状态字=0x%04X，模式=%d，位置=%d inc",
                         transport.read(Object.STATUSWORD), transport.read(Object.MODE_DISPLAY),
                         transport.read(Object.ACTUAL_POSITION))
            else:
                # 在发送任何 NMT/控制指令前检查请求值，避免无效请求使能电机。
                motion.validate(settings, transport.read(Object.ACTUAL_POSITION))
                transport.start()
                controller = ProfilePositionController(transport, settings)
        else:
            LOG.info("模拟运行：不连接 CAN，不控制真实电机")
            motion.validate(settings, 0)
            controller = ProfilePositionController(SimulatedDrive(), settings)
        if controller is not None:
            controller.prepare()
            actual = controller.move(motion)
            LOG.info("%s完成，实际位置=%d inc", "实机定位" if args.execute else "模拟定位", actual)
    except KeyboardInterrupt:
        LOG.error("收到中断，将尝试暂停驱动器")
        result = 130
    except Exception as exc:
        LOG.error("运行失败：%s", exc)
        result = 1
    finally:
        if controller is not None and controller.enable_started:
            try:
                controller.halt()
                LOG.info("已执行 Halt；驱动器保持使能，未自动去使能")
            except Exception as exc:
                LOG.error("暂停失败，不能确认电机停止，请使用现场停止措施：%s", exc)
                result = result or 1
        if transport is not None:
            try:
                transport.close()
            except Exception as exc:
                LOG.error("关闭 CAN 连接失败：%s", exc)
                result = result or 1
    return result
