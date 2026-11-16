# 三台Kinco、RS06、RS00和两台EL05的七轴PP往返

七轴在同一CAN总线运行。前三台Kinco维持每程600圈、600 RPM；四台RobStride均为
每程100圈、100 RPM、加减速各2秒。七轴按共同时间规划整数轮廓，全部到位并确认停稳后才换向。
短恢复段按各轴实际剩余距离降低速度以协调到达，配置速度是上限。

| 节点 | 型号 | 每圈位置 | 每程位移 | 速度原始值 | 加速度原始值 | 目标RPDO | TPDO1 / TPDO2 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1/2/3 | Kinco iSMD | 65536 inc | 39321600 inc | 按Kinco单位 | 按Kinco单位 | 共享0x50A控制字 | 保留已核对TPDO1 / 0x281～0x283 |
| 4 | RS06 | 16384 inc | 1638400 inc | 1000 | 500 | 0x504 | 0x184 / 0x284 |
| 5 | RS00 | 16384 inc | 1638400 inc | 1000 | 500 | 0x505 | 0x185 / 0x285 |
| 6 | EL05 | 16384 inc | 1638400 inc | 1000 | 500 | 0x506 | 0x186 / 0x286 |
| 7 | EL05 | 16384 inc | 1638400 inc | 1000 | 500 | 0x507 | 0x187 / 0x287 |

## 设备资料与适用范围

新增原始资料为`RS00使用说明书260428.pdf`、`EL05使用说明书2600428.pdf`，文件原样保留。
两份手册§5.3、§5.4、§5.6均说明：PP模式1，控制字15下更新绝对607A即可运动；
位置16384 inc/圈、速度0.1 RPM、加速度0.1 RPM/s；状态/模式/位置/速度为6041/6061/6064/606C。
因此复用已核对的RobStride PP流程，不使用Kinco bit4/bit12握手或bit8 Halt。
用户已确认RS00和EL05共用原有`RS06.eds`；新节点仍各自核对1018厂商/产品标识，
EDS身份通用，不能据此自动认定实际型号和固件兼容。

力矩不能按相同物理值照搬：各型号§5.6的6071原始1000分别代表RS06 11 N·m、
RS00 5 N·m、EL05 2 N·m。模板原始100分别约1.1、0.5、0.2 N·m；它们是独立的初始配置，
须按每轴负载核对。EL05手册§1.3列额定负载1.8 N·m，而CANopen §5.6明确用2 N·m作
6071比例，本程序采用后者解释通信值，并记录此区别，不将额定值或峰值替代通信换算。

各手册§4.2.10说明`acc_stutus=0`时加速度兼作减速度；须通过厂家方式确认，
不猜测CANopen 6084单位。控制字11停止及6099:01看门狗沿用各型号§5.3，
必须验证实际负载响应、失能行为与通信丢失保护。
新型号的同步门控、初始化SYNC及TPDO重映射行为尚未实机验证，不能把虚拟从站结果视为固件认证。

## 配置与命令

安装和测试依赖沿用README，无新增依赖。七轴使用独立入口和配置，不改写现有单轴、三轴和四轴配置。

```bash
python -m kinco_pp.seven_axis_reciprocate --legs 2
# 等价示例/安装后的命令：
python examples/seven_axis_reciprocate.py --legs 2
kinco-pp-seven-axis --legs 2
```

七轴Kinco模板为`config/seven_axis.kinco.example.json`，RobStride模板为
`config/seven_axis.robstride.example.json`，后者由`defaults`及四条`axes`组成。
`axes`依次为RS06、RS00、EL05、EL05，逐轴必须显式提供型号、节点、固定起点/远端及软件限位。
节点和COB-ID可独立配置，程序拒绝重复节点、重复目标ID或不同总线；新增三轴的完整位移和
速度须与同一文件的RS06一致。`--turns`、`--velocity`等原参数只影响前三台Kinco。

```bash
# 已有四轴现场配置时，复用其中三台Kinco原起点和限位：
cp -n config/local.four_axis.kinco.json config/local.seven_axis.kinco.json
cp -n config/seven_axis.robstride.example.json config/local.seven_axis.robstride.json
```

将现有`config/local.rs06.json`的节点4原起点、远端、限位和负载/通信参数填入七轴配置的
RS06条目，保持端点不变。节点5/6/7分别填写已确认起点，各自远端为起点加1638400 inc；
模板起点0只供模拟，不代表实机当前坐标。不能扩大软件限位来容纳未知当前位置，不能模式6归零。
各轴独立保留`hardware_verified`、`absolute_reference_verified`、`synchronous_pdo_verified`，
模板全部为false；确认标志不能从节点4复制到新增轴。

```bash
# 只读检查全部七轴身份、SYNC/目标PDO冲突、端点和完整行程可行性：
python -m kinco_pp.seven_axis_reciprocate \
  --config config/local.seven_axis.kinco.json \
  --robstride-config config/local.seven_axis.robstride.json --check

# 逐轴确认坐标、限位、负载、停止方式、PDO通道及固件后才执行：
python -m kinco_pp.seven_axis_reciprocate \
  --config config/local.seven_axis.kinco.json \
  --robstride-config config/local.seven_axis.robstride.json --execute --legs 2 \
  --csv /tmp/seven-axis-feedback.csv --output /tmp/seven-axis-result.json
```

`--check`不发送NMT、PDO配置、使能或SYNC；它不能证明实际TPDO周期/映射能力和组外总线独占。
不传`--legs`或`--duration`则持续往返；`--start-direction reverse`先回原固定起点。
纯模拟以各轴配置起点初始化，不连接CANable。

## 同步、反馈、异常处理和报告

提前通过SDO完成全部轴参数及使能确认，RobStride保持当前位置。全部RobStride进入运行态后
先共同初始化SYNC，此时未装载新目标。随后装载四条独立目标RPDO和一条Kinco共享控制字RPDO，
最后一条共同SYNC释放全部七轴，触发后立即维持设备原周期SYNC。
任何目标发送失败都进入联停，不发SYNC释放部分装载的运动目标。
发送余量仍采用各轴`sync_settle_s`的最大值；这不是硬实时发送完成保证，七轴负载须重新实测。

运动期全部七轴使用TPDO反馈。RobStride TPDO1为6041(16)+6064(32)+6061(8)，
TPDO2为606C(32)，同步类型1；两条反馈流分别按`sdo_timeout_s`检查首帧和缓存时效，
缺失/过期不降级SDO。配置、使能、停止及停稳复核仍用SDO。
任一轴故障或通信失败时，向全部已开始使能的轴请求停止；一轴失败仍处理其余轴。
确认全组停稳后才逐项恢复RPDO、TPDO和SYNC配置，不能确认则保留现场映射并报告错误。

CSV和JSON反馈按配置顺序覆盖节点1至7；`axis_recent_coordination`分别报告模型时间差及
主机观察到的到位时间差，二者都不是实机机械同步精度。
`robstride_counts`按节点和型号分别记录到位段、完整单程和完整往返；恢复段与零位移等待不混入完整行程。
整组`completed_full_legs`只在全部轴从相反固定端点出发时累计，三台Kinco已完整但某新增轴
处于中途的首段仍算恢复段。所有固定端点在重启后复用。

软件与虚拟CAN测试命令见README，新增回归覆盖七节点共同SYNC、七轴运动期禁止反馈SDO、
最后轴到位屏障、节点6丢速度TPDO、节点7目标发送失败、最后轴映射拒绝回滚和停止失败隔离。
本次未运行真实七轴；实际节点型号、固件、TPDO反馈周期和机械同步性能须另行验证。
