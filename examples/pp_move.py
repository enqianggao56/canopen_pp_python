"""单次 PP 定位示例；安装项目后运行，参数与 kinco-pp 命令相同。

默认模拟：python examples/pp_move.py --relative --target 1000
真实运行：先完成 README 的配置核对，再显式添加 --execute。
"""

from kinco_pp.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
