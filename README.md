# Kinco CANopen PP Python

使用 Python，通过 CANopen SDO 控制 Kinco 伺服电机运行于 PP（Profile Position，轮廓位置）模式。包含中文注释、命令行示例、独立模拟器及虚拟 CAN 协议测试。

当前接入目标为 **iSMD60D-040-DQBK-AA000 + CANable，节点 1，1 Mbit/s**。已完成软件测试及一次实机正向 600 圈相对定位，详见[实机验证记录](docs/hardware-validation.md)。配置和机械运动范围须结合每次实机状态确认。

## 目录

```text
src/kinco_pp/        配置换算、SDO 通信、PP 控制、模拟器和命令行
examples/pp_move.py  单次定位入口
config/             JSON 配置示例；本地 local*.json 不提交
tests/              单元测试与 CANopen 虚拟总线测试
motor.eds           原始 Kinco EDS
```

## 安装与模拟运行

要求 Python 3.8+，在项目根目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m kinco_pp --relative --target 1000 --velocity 1 --acceleration 1 --deceleration 1
# 安装后的等价入口：
kinco-pp --relative --target -1000
python examples/pp_move.py --relative --target 1000
```

不加 `--execute` 时，全部使用模拟驱动器，不连接 CAN 或驱动真实电机。若 Debian/Ubuntu 缺少 `ensurepip`，需先安装对应 Python 版本的 `python3-venv` 软件包。仅模拟控制流程也可直接运行 `PYTHONPATH=src python3 -m kinco_pp`。

## CANable 与总线准备

示例配置假设 CANable 使用 **candleLight 固件**，由 Linux `gs_usb` 驱动呈现为 SocketCAN 接口 `can0`。插入适配器后，使用以下命令检查和设置接口：

```bash
ip -details link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details -statistics link show can0
```

SocketCAN 的波特率由系统接口配置决定，JSON 中的 `bitrate` 不会替代上述系统设置。接口名称可能是 `can1`，请修改 `channel`。

如果 CANable 呈现为 `/dev/ttyACM0`，它可能使用 SLCAN 固件，不能直接套用 `can0` 配置。可在安装相应串口依赖后使用 `interface: "slcan"`、`channel: "/dev/ttyACM0"`；需确认适配器固件和串口权限。不要同时打开 SLCAN 和 SocketCAN 去控制同一适配器。

按照手册 §9.2 接线并确认 CAN_H、CAN_L、参考地及两端终端电阻；驱动器节点和波特率也必须设置为 **1、1 Mbit/s**。退出其他可能控制同一节点的调试软件。

## 配置与只读检查

```bash
cp config/servo.example.json config/local.json
kinco-pp --config config/local.json --inspect
```

`--inspect` 只进行 SDO 读取，不发送 NMT 启动、使能或运动指令。它读取 0x1018 身份、0x6041 状态、0x6061 模式及 0x6064 位置。身份记录按 0x1018:00 的实际条目数读取，可选序列号缺失时返回 `None`，厂商及产品标识仍严格核对。正常实机执行前还会再次核对身份。

| 配置项 | 含义与需要核对的内容 |
| --- | --- |
| `encoder_counts_per_rev` | 通信换算分辨率；按手册表 8-2 的注释和表 9-11 示例取 **65536**，不要根据编码器标注的位数直接计算；仍需核对实机固件适用性 |
| `expected_vendor_id` / `expected_product_code` | 示例 768 / 17988 来自 EDS；启动时配置、EDS 和实际 0x1018 必须一致 |
| `min_position_inc` / `max_position_inc` | 允许的起点、终点范围，单位 inc；示例 ±100000 不是机械安全范围 |
| `max_velocity_rpm` / `max_acceleration_rps2` | 软件速度、加减速度上限；示例 30 rpm / 10 rps/s，需按负载调整 |
| `position_tolerance_inc` | 到位校验允许的位置误差；不能大到掩盖本次运动 |
| `zero_velocity_tolerance_rpm` | 零速反馈容差；默认 0，实机反馈噪声需按测量设置，不能单凭速度容差认定停稳 |
| `standstill_duration_s` | 速度低于容差且位置变化不超过位置容差的连续确认时间；必须小于状态和运动超时 |
| `sdo_timeout_s` / `state_timeout_s` / `motion_timeout_s` | 单次 SDO、状态握手和到位等待超时，单位秒 |
| `hardware_verified` | 人工确认型号、固件、EDS、单位、坐标范围、接线及停止方式后才改为 `true` |
| `absolute_reference_verified` | 已建立并核对绝对坐标原点后才改为 `true`；代码不自动回零 |

**资料差异：** EDS 元数据写的是 `FD driver`，手册和实际目标为 iSMD。EDS 中还存在与手册显示单位不同的默认值。保留原始文件，按手册 §7.3 换算后显式写入运动参数。身份数值一致不能证明型号和固件完全适用；不一致时停止，并获取匹配资料，不能仅修改预期身份以绕过检查。

## 实机单次定位

确认机械运动范围、负载、抱闸和现场停止措施后，在本地配置中完成上述核对。先用小位移、低速度运行：

```bash
kinco-pp --config config/local.json --execute --relative --target 1000 \
  --velocity 1 --acceleration 1 --deceleration 1 --verbose
# 仅在确认绝对原点后使用：
kinco-pp --config config/local.json --execute --absolute --target 2000
```

`--target` 为驱动器 **inc**，相对模式下是位移，绝对模式下是坐标；不是角度或毫米。速度为 rpm，加减速度为 rps/s。不自动推断机械传动比或电子齿轮比。

控制流程依据随附手册 §5.1、§5.4、§7.3、表 9-11：

1. 检查身份及运动参数，向节点 1 发送 NMT 启动。
2. 按 DS402 状态转换设置 PP，先回读 0x6060，再在 Halt 下使能并确认 0x6061 为 1；实机固件仅在使能后更新模式显示，确认前不触发目标。
3. 换算并写入 0x6081/0x6083/0x6084，再写入 0x607A。
4. 使用控制字 bit4 上升沿触发，等待状态字 bit12 置位，再复位 bit4 并等待 bit12 清零。
5. 同时检查 bit10 和实际位置，确认本次到位。
6. 正常完成、异常和 Ctrl+C 均尝试 Halt，联合读取 0x606C 和实际位置确认持续静止，随后关闭通信。

程序设置 0x605D=1，Halt 按 0x6084 减速，并**保留使能锁轴，不自动去使能**；不会保存参数到 EEPROM。再次运行时，可复用已静止且处于 PP Halt 的使能状态，避免先松轴；其他已使能状态拒绝接管。SDO 超时不自动重发，防止重复相对位移。

Halt 是通信控制，不能代替硬件急停或 STO；通信断开或进程被强制终止时，软件无法保证停机。本实现没有自动配置驱动器通信丢失保护（手册 §9.4 的相关默认设置可能不处理断线），实机使用前需配置并验证现场保护。禁止用未确认停止方式的悬垂负载直接试运行。

## 测试与开发

```bash
python -m unittest discover -s tests -v
python -m compileall -q src examples tests
```

测试使用标准库 `unittest`，覆盖换算、状态切换、PP 应答、重复相对运动、负坐标、旧到位位、超时、故障和中断清理。协议测试采用 `canopen.LocalNode` 与 `python-can` 的 `virtual` 后端，不访问 CANable。模拟结果不等同于真实电机验证。

模块和函数提供中文说明，关键步骤标明索引、位定义及单位。控制器同步执行，不支持并发控制同一节点或实时多轴插补。

通信库使用官方接口：[CANopen 网络与节点](https://canopen.readthedocs.io/en/stable/network.html)、[SDO 访问](https://canopen.readthedocs.io/en/stable/sdo.html)、[SocketCAN 配置](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html)。

## GitHub 管理

远程仓库为 <https://github.com/enqianggao56/canopen_pp_python>。每次完成代码变更后，先运行适用检查，再检查差异、提交相关文件并推送 `main`。不提交本地配置、凭据或日志，不强制推送。提交示例：`feat: 添加 PP 模式位置控制`。
