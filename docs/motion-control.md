# 通信与四轮速度控制 v1.1.2

实现日期：2026-11-07；低速控制更新：2026-11-08。硬件：C30D STM32F407、四台 MG513P10-12V GMR、普通轮四驱差速。HAL/FreeRTOS/BSP 保持 C；应用 C++17；Jetson 工具为 Python3 + pyserial，不依赖 ROS。此版本不是已完成落地导航/负载调参的固件。

## 分层与所有权

- `motion.hpp/.cpp`：独立 PI、目标斜坡、反转过零、差速分解、超时状态机。借用唯一四轮对象，不分配内存。
- `motion_service`：通信到控制任务的单槽邮箱。速度覆盖旧速度；未执行的管理请求不能被速度覆盖；STOP 优先覆盖邮箱并立即关闭寄存器输出。
- 控制任务在500Hz更新编码器后执行运动服务，独占所有 Motor API。四轮有界计算/寄存器更新与 STOP 互斥；临界区不做串口、等待或分配。`navbot_motion_max_cycles/168` 给出实测最大微秒数。
- 通信任务200Hz解析协议，只提交请求/复制快照；反馈和控制状态均100Hz，健康状态10Hz。串口DMA不阻塞控制。
- IWDG仅由执行中的控制任务喂狗，标称约500ms，实际受LSI误差影响。调试冻结位清除，长时间停核会复位。复位后PWM保持禁用，不恢复命令/使能/几何配置。

## 参数、坐标与限制

协议顺序为 LF(C)、LR(D)、RF(B)、RR(A)。未配置时是既有电气正方向；配置 `sign[4]` 后，目标、反馈和累计计数统一变为该配置的逻辑方向。STOP不清计数；配置改变会改变计数符号，主机必须重建基线。

参考硬件归档 `hardware/releases/1.2.0/NavBot-Hardware-1.2.0.zip` 内 `NavBot-V16.5.FCStd`：四个85mm轮安装基准CAD坐标为 (±114, ±67, 42.4998)mm，CAD的Y为前后。初始半径0.0425m、名义轮距0.228m、轴距0.134m；这些是模型名义值，不是承载有效滚动半径或侧滑有效轮距标定。2026-11-02电机装配复查也使用85mm轮。电气前进极性必须实测，不能从CAD推断。2026-11-07用户现场确认 C/A正转向前、B/D正转向后，故逻辑前进sign按LF/LR/RF/RR为 `[1,-1,-1,1]`。

编码器暂按500PPR×4×10=20000 counts/rev，未做外部转速计/整圈标定。车体X前Y左Z上，正wz左转。运动指令不提供vy。

- 独立轮速：±12rad/s；车体vx：±0.3m/s，wz：±1.5rad/s。
- 逆运动学：(vx∓wz×track/2)/radius；超过轮速限制时四轮同比例缩放。
- PWM固定限制±200千分比；轮目标变化限制20rad/s²；反转先减到零，再等待实测旧方向速度降至0.5rad/s以内。
- 各轮独立 PI + 静摩擦/速度前馈，输出单位为占空比千分比；速度反馈一阶滤波时间常数前轮2ms、后轮0.5ms。参数按 LF(C)、LR(D)、RF(B)、RR(A)：静摩擦 `[45,20,30,15]`，速度前馈 `[9,9,9,8]`，Kp `[40,160,40,160]`，Ki 均60。这些是架空实测估计，并非负载标定值。
- 起转补偿初值 `[90,135,85,140]`，100ms后以500千分比/秒递增，始终≤200；起转阶段输出上升限速1000千分比/秒、积分冻结。连续20ms有效同向反馈确认起转；300ms内未确认则锁定故障。补偿随小目标衰减，不给零目标加力。
- 稳速阶段直接应用有幅值/方向约束的 PI 输出，不再施加起转上升限速，避免减力快、加力慢导致掉速。条件积分同时处理幅值、方向和起转限制，积分限幅±100。零目标斜坡归零后滑行并清积分，不用反向PWM纠正超速。
- 目标绝对值≥1rad/s且同向运行时，连续30ms低于0.25rad/s会尝试有界重起转；一次使能最多3次，再次失速锁定故障。新速度包不重置次数/起转期限。此诊断仅代表编码器低速，不等同外部堵转。
- STOP、超时及故障立即撤销PWM并滑行，不承诺机械瞬停；正常零速目标走斜坡。本版不自动使用电气制动。
- 无电流传感数据，不声称堵转/过流保护；PD3未作为已验证硬件急停。未开放PWM串口入口。

## 协议

沿用v1通用帧、小端逐字段编码、CRC16、session/seq与旧时间查询。固件号0x00010102；caps=0x37（raw/attitude/time/wheel-motion/configurable-body），不再声明bench-disabled。motor_present_mask=15表示配置四路，不代表自动检测出插接电机。

| ID | payload（按顺序） | 行为 |
|---|---|---|
| 0x10 CMD_VEL | vx:f32,wz:f32,valid_ms:u16，10B | 必须先配置几何/方向并使能 |
| 0x11 SET_ENABLE | enabled:u8，1B | 1申请使能；0等同STOP |
| 0x12 STOP | 空 | 立即关PWM；控制任务撤销许可；清空待执行请求 |
| 0x14 CMD_WHEELS | omega:f32[4],valid_ms:u16，18B | 已使能；用于独立轮速测试 |
| 0x31 CONFIG_GEOMETRY | radius:f32,track:f32,sign:i8[4]，12B | 仅Disabled；半径.02～.2m、轮距.1～1m、sign为±1；RAM配置 |
| 0x20 WHEEL_STATE | count:i64[4],omega:f32[4],valid:u32，52B | 最新2ms控制采样值，100Hz发送；rad/s |
| 0x23 STATUS | 原80B保持布局 | motion_enabled/wheel_valid改为实际值 |
| 0x24 MOTION_STATE | 见下表，80B | 100Hz，执行结果及PI诊断 |

0x24前32字节依次为8个u32：state、fault、applied_seq（最后应用的速度请求）、command_age_ms、configured、request_seq（最后处理的邮箱请求）、result（0已执行/4拒绝）、valid。随后target:f32[4]、ramp:f32[4]、duty:i32[4]。target/ramp单位rad/s；duty为逻辑方向千分比。控制状态0x24和轮状态0x20的时间戳均为实际控制采样时间。

state：0 Disabled、1 Enabled、2 Timeout、3 Fault。fault：1使能失败、2反馈/控制周期异常、3电机写入失败、4起转超过300ms、5重复低速恢复次数耗尽。基础模块失败还可能进入现有全局panic并由IWDG复位；不能依赖一定收到最后一帧故障遥测。

ACK：0请求入邮箱或STOP已切输出，**不表示达到速度**；1不支持、2长度错、3重复/旧序号、4状态禁止、5参数非法、6管理请求忙。管理请求应等0x24匹配request_seq/result后继续；连续速度可能覆盖中间seq，跟踪applied_seq而不要求每条都执行。命令拒绝不刷新有效期。0x13清故障暂不支持，返回4；硬件故障必须排查后复位重初始化。

有效期50～250ms，推荐200ms。从MCU接收时起计，邮箱等待不能延长寿命；控制按2ms周期检查。超时撤销使能，新的速度或使能请求不能直接恢复，须STOP确认、显式使能、新速度。每次使能给200ms首命令窗口。不同session的HELLO强制STOP并清邮箱；重新打开串口应换session。

发送帧版本仍为1：未改变既有字段布局，新增ID/明确能力协商。旧台架0x10/11/31从未定义可执行payload；客户端必须检查新固件号/能力。

## Jetson程序

`tools/chassis_serial.py`：基础编码解码、真实轮速解析；只读验证模式兼容新固件。旧`--test`为台架专用，新固件使用`motion_protocol_test.py`。

`tools/motion_host.py`：无ROS依赖的`VelocityBridge.on_twist(vx,wz,received_at)`模拟订阅回调；发送时检查上游消息年龄，旧消息不会被重新赋予完整租期。上游超过150ms不更新发送STOP并锁定桥接对象，不自动重使能。实际ROS适配必须先处理消息时间戳/时钟映射，不能把积压消息的到达时刻当作新指令。

测试程序默认1.5秒、最多5秒；异常和正常结束均尝试STOP，主机彻底失联由MCU租期兜底。仅用于已确认架空的台架，支持无人值守短测，异常会停机。首次独立轮测试不要配置方向，以确认电气正方向。车体测试必须明确传入实测sign。

```sh
PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6D001621-if00
python3 tools/motion_host.py --port "$PORT" --mode wheel --wheel 0 --omega 6 --timeout --output wheel-c.json
# 本车2026-11-07已现场确认的前进极性；更改接线/装配后重新验证。
python3 tools/motion_host.py --port "$PORT" --mode body --signs 1 -1 -1 1 --vx 0.15 --wz 0 --seconds 1.5 --output body-forward.json
python3 tools/motion_protocol_test.py --port "$PORT" --output protocol.json
bash scripts/test-motion.sh
```

当前工具读取并记录四轮角速度；body模式按左右两轮均值换算车体vx/wz，记录目标与窗口平均反馈。未实现ROS话题发布、TF/EKF或落地里程计节点。Jetson未安装ROS时可以验证相同串口控制链，不能将此测试称为实际ROS集成测试。

参考：[ROS control_toolbox条件积分抗饱和](https://control.ros.org/jazzy/doc/control_toolbox/doc/index.html)、[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。

## 本次实机结论

两轮核心测试通过：四轮独立6rad/s指令与真实反馈；Jetson模拟ROS回调发送车体前进/后退/转向指令。详细结果见 `validation/motion-control.txt`。额外0.6rad/s极低速转向未达到初始容差（反馈约0.368rad/s），保留失败原始记录；低PWM起转滞后需要后续逐轮死区/前馈及PI调参，未通过提高上限或放宽容差掩盖。所有结果为架空编码器换算，不代表落地实际车速或侧滑标定完成。

2026-11-08低速调参：已修复稳速阶段不对称PWM限速造成的延迟响应，最终结果及尚存速度波动见 `validation/pi-tuning.txt`。上述早期0.368rad/s结果作为历史基线保留。

最终回归：±0.6rad/s转向各两次、每次5秒，反馈均值+0.616/+0.618和−0.606/−0.607rad/s；四轮持续起转40～195ms。±0.15m/s直行各3秒，反馈+0.152/−0.153m/s。六次均通过超时停机、零输出、零串口错误检查。后轮瞬时速度标准差仍约0.43～0.52rad/s；未证明消除全部波动或声音，未验证低于1rad/s轮速或落地负载。

后轮二次优化（1.1.2）：降低滤波延迟并提高后轮比例反馈，同时降低静摩擦前馈，起转参数和所有保护保持不变。对照采用实际遥测间隔内累计计数差分计算轮速，避免将100Hz采样最新2ms速度的混叠误判为实际改善。结果见 `validation/rear-low-speed.txt`。

## 当前 IMU 验收边界（2026-11-08）

电机通信/速度回归通过不代表 IMU 已通过验收。复查1.1.2的 `build/rear-tuning/rear-backward.json`：0x21原始IMU共463帧，334有效、129无效；0x22姿态231帧全部无效。末次状态记录 imu_error=2（总线错误历史）、imu_recoveries=13、calibration_state=0。其余四次最终转向记录的姿态同样均无效。

这些记录说明运动测试期间存在IMU采集/校准问题，尚未定位根因；不能据此断言由电机干扰或控制周期调整导致。随后两次SSH连接被Jetson关闭，未完成新的静止状态复查。接收端必须检查valid、时间戳与校准状态，不得将无效四元数用于融合。芯片到车体安装变换、绝对航向及正式ROS适配仍未完成。
