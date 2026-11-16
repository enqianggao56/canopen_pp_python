# Kinco CANopen PP Python

使用 Python，通过 CANopen SDO/RPDO 控制 Kinco 伺服电机运行于 PP（Profile Position，轮廓位置）模式。支持单轴定位及三轴共同启动、共同时间规划，包含中文注释、命令行示例、模拟器及虚拟 CAN 协议测试。

当前接入目标为 **iSMD60D-040-DQBK-AA000 + CANable，1 Mbit/s**。已完成软件测试、节点 1 的实机正向 600 圈定位、节点 1/2/3 的三轴小位移定位，以及三轴各 150 圈、速度指令 600 RPM 的实机运行，详见[实机验证记录](docs/hardware-validation.md)。配置和机械运动范围须结合每次实机状态确认。

## 目录

```text
src/kinco_pp/        配置换算、SDO 通信、PP 控制、模拟器和命令行
examples/pp_move.py  单次定位入口
examples/three_axis_move.py  三轴定位入口
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

## 三轴同步运行

三轴共用一条 CAN 总线。默认配置 `config/three_axis.example.json` 使用节点 1/2/3，三轴相对位移分别为 1000、2000、-1500 inc。已在用户确认当前机械范围后完成这组实机运动；轮询观察到的到达时间差约 10.8 ms，详情及测量限制见[三轴验证记录](docs/hardware-validation.md#三轴小位移实机验证)。示例仍默认关闭实机执行，不代表适用于其他机械状态。

```bash
python -m pip install -e .
python -m kinco_pp.group_cli
# 安装后的等价命令与示例：
kinco-pp-group --config config/three_axis.example.json
python examples/three_axis_move.py
```

配置中的 `defaults` 是各轴共用的设备设置；`axes` 内的 `name`、`node_id`、`motion` 描述每个轴，也可覆盖共用设置，例如各自的 EDS、分辨率、软件运动范围和零速容差。每轴 `motion` 使用 `target_inc`、`velocity_rpm`、`acceleration_rps2`、`deceleration_rps2` 和 `relative`。三个节点 ID 必须不同，总线接口及波特率相同，且统一使用相对定位或绝对定位；绝对方式须逐轴确认原点。

同步流程如下：

1. 在任何轴使能前核对全部身份、目标和软件限值。
2. 暂存各轴原 RPDO 配置，在预操作状态将指定槽位映射为同一 COB-ID 下的 16 位控制字 0x6040；配置后逐项回读。
3. 三轴保持 Halt，逐轴进入 PP 并装载速度、加减速度和目标，确认全部就绪。
4. **只发送一条共同 RPDO 帧**，三个轴响应同一 bit4 上升沿；再监控全部应答、状态和目标位置。
5. 正常结束或任一轴异常时，全组请求 Halt，并向所有已开始使能的轴发送 SDO 暂停请求后逐轴确认停稳。一轴失败也继续处理其余轴；确认全组静止后恢复原 RPDO 配置，不写 EEPROM。无法确认停稳则保留映射并报告错误。

此处的共同启动使用异步 RPDO（类型 254），不要求周期性 SYNC 主站。原理与[CANopen 官方 PDO 文档中的多设备共同启动示例](https://canopen.readthedocs.io/en/stable/pdo.html)一致；驱动器内部处理时刻仍有差异，不能据此承诺硬实时精度。

为尽量同时到位，先按实际整数单位计算各轴最快梯形/三角轮廓，选最长时间作为共同时间。短时轴降低速度和加减速度，再按设备整数分辨率重新求解巡航速度、估算到达时差，参数不超过原请求上限。可在配置顶层设置 `"duration_s": 5.0` 指定更长共同时间；不可小于最快可行时间。零位移轴保持位置，不参与到达时间差计算。模型假设起终点静止，未补偿负载、反馈单位误差或驱动器动态差异；这是 PP 单段协调定位，连续轨迹插补需要另行实现。

三轴命令会分别输出共同规划时间、量化后的模型时间差，以及轮询观察到的到达时间差。**模拟器不模拟机械时间，轮询结果也包含主机及 SDO 延迟，二者都不是三轴实机同步精度。**

实机接入时，先复制配置并逐轴核对：

```bash
cp config/three_axis.example.json config/local.three_axis.json
kinco-pp-group --config config/local.three_axis.json --inspect
# 核对全部配置与 PDO 通道后才执行：
kinco-pp-group --config config/local.three_axis.json --execute
```

- `rpdo_number` 默认 4，需确认三个从站的该槽位都可临时重映射。
- `shared_cob_id` 默认 1280（0x500），需在整条总线上预留，其他节点与主站不能使用；代码检测组内其他 PDO 的冲突。
- 完成上述确认后设置顶层 `pdo_configuration_verified: true`，并逐轴设置 `hardware_verified: true`；默认均为 `false`。
- 若任一驱动器不支持映射或回读不一致，会在运动触发前失败并尝试恢复已改动配置。
- 节点必须由当前程序独占控制；Halt 与通信异常的现场保护要求同单轴模式。

## 三轴持续往返测试

`kinco-pp-reciprocate` / `python -m kinco_pp.reciprocate` 用于三轴 PP 正反往返测试，默认每轴正转 600 圈、反转 600 圈，速度 600 RPM，加速和减速各 2 秒。换算后的加减速度为 **5 rps/s**，写入 0x6083/0x6084 的整数均为 5368。默认持续运行，直到 Ctrl+C；每程先停稳，再换向。

```bash
# 完整往返一次的模拟测试，不连接硬件：
PYTHONPATH=src python3 -m kinco_pp.reciprocate --legs 2
# 安装项目后等价入口：
kinco-pp-reciprocate --legs 2
```

配置模板为 `config/reciprocate.example.json`。先复制为 `config/local.reciprocate.json`，核对三个节点、实际运动范围、零速反馈容差、PDO 槽位与共享 COB-ID，并完成硬件核对标志。每轴的 `origin_position_inc` 固定往返起点，远端为该起点加单程圈数对应的 inc；示例固定起点为0，实机需填写已经确认的端点。此字段只定义软件目标，不重设编码器坐标或驱动器零点。`axes[].motion` 仅作首程参数示意，实际软件速度及加减速度上限仍必须允许请求值。

```bash
# 在已确认600圈正反运动范围的实机上观察120秒，到时减速暂停：
python -m kinco_pp.reciprocate --config config/local.reciprocate.json \
  --execute --turns 600 --velocity 600 --accel-time 2 --decel-time 2 \
  --duration 120 --csv /tmp/reciprocate-feedback.csv --output /tmp/reciprocate-result.json
# 持续往返：省略 --duration；Ctrl+C 请求全组减速暂停。
```

配置固定起点后，程序跨重启复用原往返区间，按每程实际位置计算相对指令，避免误差累积；不会自动扩大软件范围。600 RPM 为 10 rps，考虑加减速后每程约 62 秒，完整往返至少约 124 秒，另有停稳确认及配置恢复时间。因此 **120 秒观察通常在反程途中结束**，到时先按设定减速度暂停，实际停止和清理可能略晚于120秒。

途中停止后可原命令重启：先前往原正向远端，首次不足600圈的距离作为恢复段，不计为完整单程。若要先继续回原起点，增加 `--start-direction reverse`；已处于所选端点时自动换向，避免空计数。报告分别记录到位段数、完整600圈单程数及完整往返数。三轴须同时配置固定起点；全部省略时保留旧版按本次启动位置锚定的行为，无法保证中断重启仍复用原区间。

可先用下列只读命令检查当前位置、原起点、远端和首次方向；它不发送NMT启动、PDO配置、使能或运动指令：

```bash
PYTHONPATH=src:/tmp/kinco-pp-deps python3 -m kinco_pp.reciprocate \
  --config config/local.reciprocate.json --check --start-direction reverse
```

坐标重置、位置超出范围或新圈数超过配置范围时仍会拒绝，不可扩大限值绕过检查。本次实机120秒观察及重启只读校验见[验证记录](docs/hardware-validation.md#往返中途停止后的重启修复)。

反馈在同一控制线程采样，CSV 可记录全程，JSON 汇总完成程数、最后方向、状态及最近样本。无限运行时内存仅保留最近4096个样本和100程摘要；CSV可选。输出文件采用独占创建，请使用未存在的路径。通信、状态、采样文件写入或停机异常会停止循环并报告错误，限时停止不能掩盖暂停失败。

## 测试与开发

```bash
python -m unittest discover -s tests -v
python -m compileall -q src examples tests
```

测试使用标准库 `unittest`，覆盖换算、状态切换、PP 应答、重复相对运动、负坐标、旧到位位、超时、故障和中断清理。三轴测试额外验证时间规划、单帧触发、节点重复/掉线、共享 COB-ID 冲突、部分映射失败回滚及全组暂停。协议测试采用 `canopen.LocalNode` 与 `python-can` 的 `virtual` 后端，不访问 CANable。模拟结果不等同于真实电机验证。

模块和函数提供中文说明，关键步骤标明索引、位定义及单位。保留单轴命令与 API；三轴使用 `ThreeAxisController`、`GroupSettings` 和 `CanopenGroupBus`。同一节点不得由多个控制器并发操作。

通信库使用官方接口：[CANopen 网络与节点](https://canopen.readthedocs.io/en/stable/network.html)、[SDO 访问](https://canopen.readthedocs.io/en/stable/sdo.html)、[SocketCAN 配置](https://python-can.readthedocs.io/en/stable/interfaces/socketcan.html)。

## GitHub 管理

远程仓库为 <https://github.com/enqianggao56/canopen_pp_python>。每次完成代码变更后，先运行适用检查，再检查差异、提交相关文件并推送 `main`。不提交本地配置、凭据或日志，不强制推送。提交示例：`feat: 添加 PP 模式位置控制`。
