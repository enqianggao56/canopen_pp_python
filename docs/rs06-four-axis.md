# 三台 Kinco 加 RS06 的四轴 PP 往返

前三轴保持节点1/2/3、每程600圈、600 RPM；RS06为节点0x04、每程100圈、100 RPM。
加减速各2秒，共同时间约62秒。正程从固定起点到远端，反程回原起点；
不是每次到位后再从当前位置向两侧各扩展100圈。四轴全部到位并停稳后一起换向。

## 资料和厂家差异

仓库保留原始`RS06.eds`和`RS06使用说明书260428.pdf`，复制自项目上一级目录，未修改内容。
依据RS06手册§5.3、§5.4、§5.6；Kinco继续依据原iSMD手册。

| 项目 | Kinco节点1/2/3 | RS06节点4 |
| --- | --- | --- |
| 每圈通信位置 | 65536 inc | 16384 inc |
| 单程位置幅值 | 39321600 inc（600圈） | 1638400 inc（100圈） |
| 速度原始值 | 0x6081=10737418 | 0x6081=1000，单位0.1 RPM |
| 加速度原始值 | 0x6083=5368 | 0x6083=500，单位0.1 RPM/s |
| PP触发 | 控制字bit4上升沿、bit12应答 | 控制字15下直接更新绝对0x607A |
| 本程序停止 | Halt减速并保持使能 | §5.3控制字11急停，需验证浪涌电压与负载响应 |
| EDS身份 | Vendor 768 / Product 17988 | Vendor 1366 / Product 17988 |

RS06的EDS列有0x6084，但手册CANopen PP步骤没有说明其作用及单位，本实现不写入该对象。
手册§4.2.10说明`acc_stutus=0`时加速度兼作减速度；此字段没有在CANopen表中给出地址，
须通过厂家支持方式确认现有固件配置。程序不猜测私有参数或切换通信协议。
`hardware_verified=true`包含对这个前提的人工核对。

EDS的ProductName为`CAN Drive Rev1.0`，身份记录一致也不能证明实机为RS06及固件完全兼容。
实际完整型号、固件、CANopen协议、节点号及1 Mbit/s必须现场核对。默认不自动切换协议或重设节点。

## 共同启动与到位

三台Kinco仍用同一个COB-ID 0x500的RPDO4接收16位控制字；RS06的RPDO4独立接收
32位绝对目标，COB-ID 0x504。四轴RPDO均配置为同步类型1，先发送两种RPDO，
再发同一条SYNC（0x80）让四轴处理命令，不按顺序调用四次单轴move。
协议依据：[CANopen SYNC](https://canopen.readthedocs.io/en/stable/sync.html)、
[CANopenNode PDO同步处理](https://github.com/CANopenNode/CANopenDemo/blob/master/tutorial/PDO.md)。

`sync_settle_s`默认3 ms，给两条命令帧预留发送时间再发高优先级SYNC。
这不是CAN硬件发送完成确认；必须结合总线负载和固件响应实测。
四轴必须由本程序独占，不能存在其他SYNC生产者或运动主站。
`--check`只读确认四轴0x1005=0x80消费者和两条预留COB-ID的组内冲突，不能证明没有组外主站。
运行时暂设四轴0x1006=0，采用事件SYNC；仅全组确认静止后恢复原PDO映射和SYNC周期。
不会写EEPROM或改变编码器原点。

三轴和第四轴按同一时间分别规划、按各自整数单位量化；短恢复段可以降低轮廓速度，
但不超过600/100 RPM上限。默认完整单程模型到达时差约0.000265秒。
JSON `four_axis_recent_coordination`分别保存每程模型时差和四轴轮询观察时差。
轮询观察值包含SDO延迟、主机调度及RS06连续到位窗口；模拟器不模拟机械时间。
`hardware_validated=false`表示软件不能据此声明实机同步验证通过。

## 配置核对

前三轴继续使用`config/local.reciprocate.json`；不要改变已确认的固定端点和软件范围。
第四轴单独复制`config/rs06.example.json`为`config/local.rs06.json`，按README运行。
`--turns`、`--velocity`、`--accel-time`、`--decel-time`只影响Kinco三轴；
RS06参数独立来自自己的配置文件，不会被600 RPM的命令行参数覆盖。

| RS06配置项 | 单位及核对要求 |
| --- | --- |
| `origin_position_inc` / `far_position_inc` | 已确认的绝对端点inc；差1638400为100圈，模板起点0仅供模拟 |
| `min_position_inc` / `max_position_inc` | 实际机械允许软件范围inc，不能扩大以绕过重启保护 |
| `velocity_rpm` / `acceleration_time_s` | 默认100 RPM、2秒；本次适配拒绝超过100 RPM |
| `torque_limit_raw` | PP必设的力矩上限，单位0.1%负载率；模板100约1.1 N·m，手册1000=11 N·m，须按实际负载核对 |
| `position_tolerance_inc` / `position_window_time_ms` | 默认25 inc、100 ms；同时校验bit10、绝对位置、零速及连续确认窗口 |
| `zero_velocity_tolerance_rpm` / `standstill_duration_s` | 默认0 RPM、0.1秒；速度反馈容差不能代替位置持续稳定 |
| `watchdog_timeout_s` | 默认1秒，§5.3的0x6099:01按20000 tick/s设置并回读；断线后由驱动器失能，须验证固件行为 |
| `target_cob_id` / `sync_settle_s` | 默认0x504、3 ms；须现场预留并验证发送余量 |
| `sdo_timeout_s` / `state_timeout_s` / `motion_timeout_s` | 默认0.3 / 3 / 90秒；完整单程和恢复段均需能在运动时限内完成 |
| `hardware_verified` | 核对型号、固件、协议、单位、范围、加减速耦合、抱闸及停止方式后才设true |
| `absolute_reference_verified` | 已确认绝对端点坐标；程序不会模式6归零 |
| `synchronous_pdo_verified` | 已核对RS06同步RPDO行为、RPDO4槽位、COB-ID及SYNC独占后才设true |

三个RS06验证标志以及原三轴`hardware_verified`/`pdo_configuration_verified`默认均关闭。
`--check`可以在验证标志未开启时只读检查，但不表示已满足全部人工核对条件。

重启保持原端点，`--start-direction reverse`先回原起点；恢复段不算完整100圈单程。
前三轴保留原完成计数，第四轴另记`rs06_completed_legs`、`rs06_completed_full_legs`及
`rs06_completed_round_trips`。第四轴在目标端点零距离等待不计作完成一程。
CSV与JSON采样按三台Kinco、RS06的顺序记录四轴，速度按各厂家单位独立解码。

## 停止与验证范围

任一轴异常、SDO失败、Ctrl+C、到达时限均进入四轴停止流程，先请求全部停止，再逐轴确认。
一轴请求失败也继续处理其他轴。RS06使用控制字11急停，不套用Kinco bit8 Halt；
厂家明确提示急停可能引起浪涌电压，必须核对电源、负载和现场停止措施。
RS06看门狗失能不是硬件急停，且不保证失能后锁轴。

若RS06停止通信失败，不再发SYNC释放可能残留的目标，继续通过逐轴SDO请求停止。
无法确认全组停止时不恢复映射，并将清理失败报告为错误；不能把限时停止伪装为成功。

软件回归和虚拟CAN协议验证不等于实机运动。当前实机只读身份检查在节点1无SDO应答时退出，
未发送使能或运动；速度、启动时差、到达时差、RS06看门狗和停止行为仍待实机验证。
