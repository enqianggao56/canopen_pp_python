"""七轴PP往返入口：Kinco 1/2/3、RS06 4、RS00 5、EL05 6/7。

复用混合轴共同SYNC、TPDO反馈和逐轴联停流程；默认纯模拟，--execute才运动。
"""

import sys
import time

from .four_axis_reciprocate import (FourAxisBus, FourAxisReciprocationRunner,
                                    SimulatedFourAxisBus, validate_topology)
from .robstride import validate_profiles


class SevenAxisBus(FourAxisBus):
    """七节点共享网络；全部RobStride独立目标RPDO，同一SYNC释放。"""

    def __init__(self, settings, profiles, sleep=time.sleep):
        """校验三台Kinco和四台RobStride配置；构造不连接或使能。"""
        validate_profiles(profiles)
        super().__init__(settings, profiles[0], sleep, extra_settings=profiles[1:])


class SimulatedSevenAxisBus(SimulatedFourAxisBus):
    """七轴独立协议模拟器，只验证控制编排，不模拟实际运动时间。"""

    def __init__(self, initial_positions=(0, 0, 0), companion_positions=(0, 0, 0, 0)):
        """以inc初始化各轴坐标，必须提供四个伴随轴位置。"""
        if len(companion_positions) != 4:
            raise ValueError('七轴模拟必须提供四个RobStride初始位置')
        super().__init__(initial_positions, companion_positions[0], companion_positions[1:])


class SevenAxisReciprocationRunner(FourAxisReciprocationRunner):
    """七轴全部到位并停稳才换向，每台RobStride分别统计完整行程和恢复段。"""

    def __init__(self, bus, settings, parameters, profiles,
                 clock=time.monotonic, sleep=time.sleep, sample_sink=None):
        """绑定七轴及共同时间规划；速度rpm、位移inc，异常由基类联停全组。"""
        validate_profiles(profiles)
        validate_topology(settings, profiles[0], profiles[1:])
        super().__init__(bus, settings, parameters, profiles[0], clock, sleep, sample_sink,
                         extra_settings=profiles[1:])


def main(argv=None):
    """独立七轴命令；配置保留逐轴确认标志，--check只读，--execute才实机运行。"""
    from .reciprocate import main as reciprocate_main
    return reciprocate_main(['--config', 'config/seven_axis.kinco.example.json',
                            '--robstride-config', 'config/seven_axis.robstride.example.json']
                            + list(sys.argv[1:] if argv is None else argv))


if __name__ == '__main__':
    raise SystemExit(main())
