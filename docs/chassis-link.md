# USART3 台架通信协议 v1.0

日期：2026-10-21。实现范围：C30D 无电机台架遥测、双向查询和软件停止确认。
沿用 NavBot 协议草案的 version=1 通用帧；HELLO firmware=0x00010000、caps.bit3 区分本台架配置。
这不是已完成运动控制的生产协议。CONFIG、使能、速度和清故障均不执行。

## 物理接口与帧

板载中间“串口3”USB → USB转串口芯片 → PD8/PD9 USART3。
460800 baud、8数据位、无校验、1停止位、无流控。DAPLink用于SWD，不作为数据串口。
实测USB串口身份 VID:PID=1A86:55D4、序列号5A6D001621；设备路径必须显式传入工具。

| 偏移 | 内容 |
| --- | --- |
| 0 | SOF：5A A5，2字节 |
| 2 | version：u8=1 |
| 3 | msg_id：u8 |
| 4 | payload_len：u16，0～256 |
| 6 | seq：u32 |
| 10 | session_id：u32 |
| 14 | timestamp_us：u64 |
| 22 | payload：N字节 |
| 22+N | CRC16：u16 |

所有多字节字段小端，float为IEEE754 binary32；逐字段编码，不发送结构体内存。
总长24+N，最大280字节。CRC16/CCITT-FALSE，poly=0x1021、init=0xFFFF、无反射/xorout，
覆盖偏移2到payload末尾。`123456789`→0x29B1。

黄金帧：STOP、seq=0x12345678、session=0xABCDEF01、timestamp=0x0102030405060708：
`5aa5011200007856341201efcdab0807060504030201948e`。

MCU逐字节解析，CRC/版本/超长错误滑动一字节重同步；不完整候选帧20ms超时后滑动重同步。
每5ms最多消费512字节，解析容量280字节。DMA循环缓冲512字节，HT/TC/IDLE回调将增量复制到4096字节软件环。
环满丢弃新块并计数，任务清除未解析内容。UART错误在通信任务中abort/restart RX，清除残留解析数据。
无动态内存；DMA、ISR和任务各自拥有缓冲/游标，只有任务操作协议和TX。所有缓冲位于普通SRAM。

TX使用1024字节DMA批次，完成前不修改；响应队列最多4个，满则计response_drops。
最多发送当前快照，不补发历史遥测；批次无法容纳计tx_drops。TX持续忙超过100ms则abort并计tx_errors。
无效输入不会锁定任务，也不会产生运动输出。持续洪泛不保证每个请求都有应答，应检查错误计数。

## 会话与时间

主机以随机非零session发送HELLO，MCU采用此session并响应，然后开始遥测。
不同session的HELLO清除旧应答与接收序号基线；同session HELLO保留序号校验。
其他旧session/零session请求静默丢弃、计rejected。
每个MCU启动用硬件RNG生成64位boot_id；RNG失败锁定禁用。重启清空session，必须重新HELLO。
主机发现boot_id改变须重建时间基线。随机身份碰撞概率极小，但不是持久递增计数。

两方向seq独立递增、u32回绕。当前会话通过CRC的请求要求：`delta = seq-last_seq` 在1～0x7FFFFFFF。
重复、倒序返回ACK(result=3)。本台架查询不缓存幂等响应，丢失应答后用新seq重试。
格式错误/未知ID的完整帧也消费seq；CRC错误不消费seq。主机用ACK的request_seq关联请求。

MCU时间由1kHz RTOS tick扩展为64位，单位us、**分辨率1000us**，不是硬件微秒采样时钟。
IMU/姿态帧时间表示最后样本的读事务起始tick，不是芯片内部转换时刻。
读取失败/数据过期时保留旧数据与旧时间，但valid=0；接收者不能把重复快照当作新样本。
轮状态是禁用占位状态，时间为生成时刻。其他帧为处理/生成时刻。

TIME_SYNC t2为通信任务本轮开始时间，含最多约一个5ms轮询周期的接收量化误差；t3为准备TX批次时间，
不是最后一位发出时间。主机t4为用户态读取并解析时刻，还含USB/OS/读超时延迟。
主机可算RTT=`t4-t1-(t3-t2)`、偏移=`((t2-t1)+(t3-t4))/2`，应过滤高延迟样本；
本实现只验证双向时间查询，**未证明满足ROS融合所需的时钟同步精度**。

关闭主机串口不会改变MCU状态；MCU可能继续向原session上报。重开时用新随机session HELLO。
没有依赖USB拔插检测或运动看门狗；本固件根本没有运动使能路径。

## 消息布局

以下字段按出现顺序紧密排列，括号为payload总字节数。

| ID | 方向 | payload |
| --- | --- | --- |
| 0x01 HELLO | 下行 | 空(0) |
| 0x01 CAPS | 上行 | request_seq:u32, boot_id:u64, firmware:u32, caps:u32, clock_resolution_us:u32, imu_hz:u16, attitude_hz:u16, wheel_hz:u16, status_hz:u16, imu_id:u8, motor_present_mask:u8, axes:u8, sample_time_kind:u8 (36) |
| 0x02 TIME_SYNC | 下行 | t1:u64 (8) |
| 0x02 TIME_SYNC | 上行 | request_seq:u32, t1:u64, t2:u64, t3:u64 (28) |
| 0x12 STOP | 下行 | 空(0)，保持电机禁用，返回ACK |
| 0x20 WHEEL_STATE | 上行 | count:i64[4], omega:f32[4], valid:u32 (52)，100Hz |
| 0x21 IMU_RAW | 上行 | valid:u32, gyro:f32[3], accel:f32[3] (28)，100Hz |
| 0x22 ATTITUDE | 上行 | valid:u32, q:f32[4] (20)，50Hz |
| 0x23 STATUS | 上行 | 下表20个u32字段 (80)，10Hz |
| 0x7F ACK | 上行 | request_seq:u32, request_id:u8, result:u8 (6) |

caps：bit0=IMU_RAW、bit1=ATTITUDE、bit2=TIME_SYNC、bit3=台架强制禁用，其他位0。
firmware=0x00010000；各频率依次100/50/100/10Hz；imu_id=0xEA为预期芯片ID，实时身份健康通过STATUS报告。
motor_present_mask=0，不声称检测到电机。axes=1：ICM20948芯片轴；sample_time_kind=1：读事务起始时刻。

轮顺序预留左前/左后/右前/右后；当前未配置通道映射或编码器比例，计数/速度全0且valid=0。
valid bit0～3将来表示独立轮有效性；当前不启动编码器，也不能用这些0积分里程计。
omega单位rad/s，count为累计编码器计数；本台架仅定义格式，未验证实际电机/编码器通路。

IMU valid=1仅在驱动有效且年龄<30ms。gyro单位rad/s，accel单位m/s²、保留重力；
IMU_RAW使用未减应用层零偏的换算值，没有磁力计、安装变换或加速度标定。
ATTITUDE valid=1同时要求IMU和姿态有效且各自年龄<30ms。
q顺序x/y/z/w，芯片坐标到Z向上局部世界坐标；六轴融合使用静态陀螺零偏，航向相对、可能漂移。
heading_epoch变化表示姿态参考重建，不能认为跨epoch连续。零偏/姿态细节见README。

ACK result：0成功、1未知/不支持ID、2长度错误、3旧/重复序号、4台架禁止。
0x10/0x11/0x13/0x31一律返回4，不解析也不执行其payload。其余未知ID返回1。
HELLO/TIME_SYNC/STOP长度错误返回2；不支持版本仅丢弃计数、不响应。

STATUS字段顺序（偏移为字段序号×4）：

| 序号 | 字段 | 含义 |
| --- | --- | --- |
| 0 | motion_enabled | 恒0 |
| 1 | wheel_valid | 恒0 |
| 2 | imu_health | 0启动、1正常、2总线错、3身份错、4配置错、5过期 |
| 3 | imu_error | 驱动保留的最后总线错误，见c30d_soft_i2c.h |
| 4 | imu_sequence | 成功采样序号 |
| 5 | imu_timestamp_ms | 最后采样u32 tick，可回绕 |
| 6 | calibration_state | 0预热、1标定中、2已标定 |
| 7 | heading_epoch | 航向参考重建次数 |
| 8 | crc_errors | 下行CRC错累计 |
| 9 | length_errors | 下行长度超限累计 |
| 10 | version_errors | 下行版本错累计 |
| 11 | parse_timeouts | 不完整候选超时累计 |
| 12 | rx_overflows | RX软件环丢块累计 |
| 13 | uart_errors | HAL UART错误回调累计 |
| 14 | tx_drops | TX批次容量不足丢帧累计 |
| 15 | response_drops | 应答队列满累计 |
| 16 | rejected | 会话/序号拒绝累计 |
| 17 | accepted | 通过会话/序号的完整请求累计，含不支持命令 |
| 18 | imu_recoveries | IMU故障后恢复累计 |
| 19 | tx_errors | TX启动失败/超时累计 |

计数器u32自然回绕，复位清零；STATUS字段5保留u32，帧头时间为扩展u64。
未实现电压、PD3开关、电流、真实电机反馈、使能、速度闭环、参数写Flash、ROS节点。
不提供不存在测量的“有效0值”。这次不冻结生产运动协议的有效期、停车斜坡和标定参数。

## 运行与验证

```sh
./scripts/test-link.sh
./scripts/test-imu.sh
./scripts/test-attitude.sh
./scripts/check.sh
./scripts/format.sh --check
./scripts/build.sh Debug 4
python3 -m venv build/comm-venv
build/comm-venv/bin/pip install -r tools/requirements-serial.txt
# 已授权烧录，且电机断开：
openocd -f openocd.cfg -c 'program build/Debug/navbot_body.elf verify reset exit'
build/comm-venv/bin/python tools/chassis_serial.py --port /dev/cu.usbserial-5A6D0016211 --test --seconds 30
# 另起OpenOCD服务后执行；会注入IMU故障并重启MCU：
openocd -f openocd.cfg
build/comm-venv/bin/python tools/chassis_bench.py --port /dev/cu.usbserial-5A6D0016211
```

主机工具不自动选择串口，Linux/Jetson/树莓派传入对应稳定设备路径即可；未验证这些平台驱动。
`--test`包含错误注入/会话回绕测试、30秒遥测及关闭重开串口。
`chassis_bench.py`额外注入Debug IMU NACK、串口噪声洪泛、MCU复位，并读取电机/CPU故障寄存器。
电脑读超时20ms，RTT不是硬件时延精度评估。
最终频率验收在不附加GDB时进行；GDB暂停会停止目标时间，不能把调试期间主机墙钟频率当作连续运行频率。
主机以独立测量窗口统计丢帧和解析错误；打开串口时残帧导致的重同步另计startup_resync_errors。
帧级日志、报告、烧录和GDB证据在build/Debug/serial-*；持久实测摘要见validation/serial-bench.txt。

参考：[ST循环DMA示例](https://github.com/STMicroelectronics/STM32CubeF4/blob/master/Projects/STM32446E-Nucleo/Examples/UART/UART_ReceptionToIdle_CircularDMA/readme.txt)、
[pySerial 3.5 API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)。
