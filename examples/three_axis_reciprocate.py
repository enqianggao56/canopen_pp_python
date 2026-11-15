"""三轴正反往返测试入口；默认模拟，实机须显式 --execute。

模拟一往返：python examples/three_axis_reciprocate.py --legs 2
实机观察两分钟：完成配置核对后添加 --execute --duration 120。
省略 --duration 和 --legs 时持续运行，Ctrl+C 请求暂停并恢复PDO配置。
"""

from kinco_pp.reciprocate import main

if __name__ == '__main__':
    raise SystemExit(main())
