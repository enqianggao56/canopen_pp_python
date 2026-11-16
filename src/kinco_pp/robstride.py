"""RS06/RS00/EL05型号配置；依据各型号260428手册§5.3、§5.4、§5.6。

三者位置/速度/加速度单位和PP流程一致，用户确认共用原始RS06.eds。
0x6071负载率换算按各型号PP表独立确定，不能用额定/峰值扭矩替换该比例。
"""

import json
from dataclasses import dataclass, replace
from pathlib import Path

from .rs06 import RS06Settings

# 各型号手册§5.6：6071原始1000分别表示11、5、2 N·m。
TORQUE_AT_1000_NM = {'RS06': 11.0, 'RS00': 5.0, 'EL05': 2.0}


@dataclass(frozen=True)
class RobStrideSettings(RS06Settings):
    """一台RobStride独立配置；继承已核对的共同单位、状态流程和固定端点保护。

    model只选择已核对手册的型号及力矩说明，不推断实机型号或修改协议。
    现有hardware_verified等标志必须逐轴核对，不能由节点号或共享EDS自动置真。
    """

    model: str = 'RS06'

    def __post_init__(self):
        """校验支持型号及父类参数；无效配置抛ValueError，不访问硬件。"""
        if self.model not in TORQUE_AT_1000_NM:
            raise ValueError('model必须为RS06、RS00或EL05')
        try:
            super().__post_init__()
        except ValueError as exc:
            raise ValueError(f'{self.model}/节点{self.node_id}: {exc}') from exc

    @property
    def torque_limit_nm(self):
        """按本型号§5.6返回6071限制对应的N·m，仅作说明，不改写电机参数。"""
        return self.torque_limit_raw / 1000 * TORQUE_AT_1000_NM[self.model]


def load_seven_axis_profiles(filename):
    """读取七轴中四台RobStride的JSON，EDS路径相对配置文件；错误直接抛出。

    defaults保存共享通信/运动参数；axes必须分别提供型号、节点、固定两端及限位。
    新增三轴速度和完整位移须与首台RS06一致，允许各轴绝对起点不同。
    """
    path = Path(filename).resolve()
    with path.open(encoding='utf-8') as stream:
        data = json.load(stream)
    profiles = []
    for axis in data['axes']:
        for key in ('model', 'node_id', 'origin_position_inc', 'far_position_inc',
                    'min_position_inc', 'max_position_inc'):
            if key not in axis:
                raise ValueError(f'每台RobStride必须独立配置{key}')
        values = dict(data.get('defaults', {}), **axis)
        # 预定义RPDO4默认ID为0x500+节点号，与Kinco共享0x50A分别预留。
        values.setdefault('target_cob_id', 0x500 + axis['node_id'])
        config = RobStrideSettings(**values)
        profiles.append(replace(config, eds=str((path.parent / config.eds).resolve())))
    validate_profiles(profiles)
    return tuple(profiles)


def validate_profiles(profiles):
    """检查四台伴随轴型号顺序及相同物理速度/完整位移；重复ID由总线拓扑校验。"""
    if [p.model for p in profiles] != ['RS06', 'RS00', 'EL05', 'EL05']:
        raise ValueError('七轴伴随配置必须依次为RS06、RS00、EL05、EL05')
    reference = profiles[0]
    distance = reference.far_position_inc - reference.origin_position_inc
    for config in profiles[1:]:
        if config.velocity_rpm != reference.velocity_rpm:
            raise ValueError(f'{config.model}/节点{config.node_id} 运行速度必须与RS06一致')
        if config.far_position_inc - config.origin_position_inc != distance:
            raise ValueError(f'{config.model}/节点{config.node_id} 完整位移必须与RS06一致')
