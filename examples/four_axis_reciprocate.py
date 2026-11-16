"""三台 Kinco 和一台 RS06 同步 PP 往返入口，默认模拟；--legs 2 为一次完整往返。"""

from kinco_pp.four_axis_reciprocate import main


if __name__ == '__main__':
    raise SystemExit(main())
