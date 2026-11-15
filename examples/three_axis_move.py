"""三轴定位示例；安装后使用 python examples/three_axis_move.py，默认模拟。

运动目标、速度、加减速度与每轴机械限值均在三轴 JSON 配置中设置。
只有显式 --execute 且完成硬件/PDO 核对，才会发出实机运动指令。
"""

from kinco_pp.group_cli import main

if __name__ == "__main__":
    raise SystemExit(main())
