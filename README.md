# navbot_body

NavBot C30D 小脑固件模板，STM32F407VET6 + HAL + FreeRTOS + C11。

截至2026-10-21，已通过 CMSIS-DAP 在实板完成 Debug 编译、烧录校验、ICM20948 六轴与温度采集、静态陀螺仪零偏标定、六轴姿态解算、OLED 显示及故障恢复测试。电机闭环、通信协议和看门狗尚未实现，电机输出保持禁用。

## 工程结构

参考本机 `/Users/lvjiaqing/ForWork/vehicle-firmware` 的 Applications / BSP / Core 分层，保留当前 C11 技术栈，未复制该工程的业务代码、C++算法、其他传感器驱动或硬件引脚。

```text
navbot_body/
├── Applications/          # 四任务周期与后续业务逻辑
│   ├── app_tasks.c
│   └── app_tasks.h
├── BSP/C30D/              # C30D寄存器级输出禁止与故障锁定
├── Core/                  # CubeMX生成的初始化、任务静态资源、ISR、薄封装
├── Drivers/               # STM32 HAL / CMSIS（组件许可原样保留）
├── Middlewares/           # CubeF4自带FreeRTOS与CMSIS-RTOS2
├── cmake/                 # 生成的工具链、HAL/RTOS源文件清单
├── scripts/               # 构建、清理、诊断、检查、格式化与符号导出
├── tools/                 # 纯静态检查与隔离再生成脚本
├── validation/            # 审查结论、依赖来源与告警验证证据
├── .vscode/               # 手动配置/构建/静态检查/附加调试入口
├── navbot_body.ioc
├── navbot_body.code-workspace
├── CMakeLists.txt
└── openocd.cfg
```

根 CMake 使用显式源文件清单，将 BSP/Application 包含路径通过 `stm32cubemx` 接口传播至 FreeRTOS、HAL 和主程序。`Core/Src/freertos.c` 持有 CubeMX生成的静态TCB/栈和线程入口；任务入口仅调用 Applications 函数。`FreeRTOSConfig.h` 中的断言可访问 BSP 声明，不依赖 Applications。

## 技术与硬件基线

| 项目 | 配置 |
| --- | --- |
| 生成器/数据库 | STM32CubeMX 6.16.1 / DB.6.0.161 |
| MCU包 | STM32CubeF4 V1.28.3，项目内复制依赖 |
| 内核 | FreeRTOS V10.3.1，CMSIS-RTOS2，ARM_CM4F端口 |
| 工具链 | CMake ≥3.22、Ninja、Arm GNU `arm-none-eabi-*`，C11、显式Thumb模式 |
| MCU | STM32F407VET6，LQFP100，Flash512KiB、SRAM128KiB、CCM64KiB |
| 时钟 | HSE8MHz，PLL8/336/2/7，SYSCLK168MHz、APB1 42MHz、APB2 84MHz |
| 定时器时钟 | APB1定时器84MHz，APB2定时器168MHz |
| HAL/RTOS时基 | TIM6_DAC 1ms / SysTick 1kHz，单独中断处理链 |
| 未启用RTC分支 | HSE÷8=1MHz，消除非法派生频率；不初始化RTC |
| Flash入口 | `0x08000000`；尚未为厂家64KiB Bootloader做偏移适配 |

新板引脚及8MHz依据见[硬件分析](../../docs/c30d-hardware-analysis.md)、新版GMR厂家源码的 `HSE_VALUE=8000000` 与 `PLL_M=8`。旧版厂家IOC为25MHz，未沿用其时钟。实板版本/晶振仍需核对。DMA缓冲只能放普通SRAM，不能放CCM。

| 功能 | 资源 |
| --- | --- |
| 电机A | PB8 / TIM10_CH1，PB9 / TIM11_CH1 |
| 电机B | PE5 / TIM9_CH1，PE6 / TIM9_CH2 |
| 电机C | PE9 / TIM1_CH1，PE11 / TIM1_CH2 |
| 电机D | PE13 / TIM1_CH3，PE14 / TIM1_CH4 |
| PWM | 10kHz，PSC0/ARR16799，CCR初始0，引脚下拉，不启动输出 |
| 编码器A/B | PA15、PB3 / TIM2；PB4、PB5 / TIM3 |
| 编码器C/D | PB6、PB7 / TIM4；PA0、PA1 / TIM5 |
| 编码器模式 | TI12，TIM2/5为32位，TIM3/4为16位，过滤15，未启动 |
| Jetson通信预留 | PD8/PD9 USART3，460800/8N1，RX DMA1_Stream1循环、TX DMA1_Stream3普通，均Channel4 |
| 调试串口 | PA9/PA10 USART1，115200/8N1 |
| 电池采样预留 | PB0 ADC1_IN8，21MHz/12bit/480周期，仅初始化 |
| 开关/按键/LED | PD3/PE0/PE8；未判断输入有效电平 |
| IMU | PB10/PB11 开漏软件 I²C；PB12 INT 保持输入；初始化前均为输入 |
| OLED | 128×64，PD14 SCLK / PD13 DIN / PD11 DC / PD12 RST，四线串行 |
| 调试 | PA13/PA14 SWD；编码器使用原JTAG复用引脚 |

C/D方向不能按通道顺序推断。A/B/C/D到实际车轮的映射、编码器极性和每圈计数、轮径、有效轮距、PI、电池阈值均待实测，不填入示例值。过滤15须结合最高编码器脉冲频率核对。

## FreeRTOS职责与边界

| 任务 | CMSIS优先级 | 周期 | 静态栈 | 后续唯一所有权 |
| --- | --- | --- | --- | --- |
| controlTask | High / 40 | 10ms | 2048B | 编码器模差分、四路PI、运动状态、PWM |
| commTask | Normal / 24 | 5ms | 2048B | 串口收发缓冲、校验/会话、固定容量消息 |
| sensorTask | BelowNormal / 16 | 10ms | 1536B | 有界IMU/ADC采集、样本时间与有效性 |
| monitorTask | Low / 8 | 100ms | 2048B | 新鲜度、栈水位、诊断及看门狗策略 |

四个应用任务均静态分配，栈合计7680B；CMSIS包装器提供Idle/Timer静态内存回调。内核仍启用heap_4（8192B）与定时器服务，不能称为全静态内核。当前维护代码不分配堆内存；采集任务独占驱动，以短临界区按值发布快照，监控任务按值读取并独占 OLED 缓冲。

任务采用绝对周期等待，tick回绕由CMSIS包装器处理；控制、通信和监控任务截止时间过期（包含正好等于当前tick）返回错误并锁定。采集任务则计数、使样本无效并重新同步周期，避免故障重试导致全局锁定。IMU 任务统计轮询/成功样本间隔与执行耗时；monitorTask 记录四任务栈水位。实测范围不等于完整 WCET 或硬实时保证。

内核初始化后检查 `osKernelReady`，任务创建检查空句柄，调度器启动若返回即锁定。故障处理不使用HAL句柄、RTOS或阻塞I/O，先关闭IRQ，再按外设时钟状态清零CCR、关闭CCER/MOE。模板没有使能API，控制任务持续保持禁用；软件输出禁止不等于已验证的驱动电平、停车能力或硬件断能。

UART/DMA中断逻辑优先级5，HAL TIM6与RTOS tick为15，NVIC分组4。数值小于5的ISR不得调用FreeRTOS API；TIM6不调用RTOS。SVC/PendSV归内核，SysTick归CMSIS包装器，TIM6归HAL。

后续使用值拷贝固定容量队列传递命令/快照：通信任务完成验证后交给控制任务，控制任务独占PI和运动状态；DMA缓冲在完成前归驱动，ISR只做有界通知；64位快照需同步。通信队列、协议、运动状态机、闭环和看门狗尚未实现；IMU 已具有初始化/采集/恢复状态机、毫秒采样时间戳和基于 DWT 的微秒间隔诊断。

## VS Code与OpenOCD

打开 `navbot_body.code-workspace` 或此目录。使用 CMake Tools、C/C++，调试可选 Cortex-Debug；不依赖STM32官方插件。未安装扩展。打开工作区不会自动配置或构建。

`arm-none-eabi-gcc`、`arm-none-eabi-gdb`、CMake、Ninja及OpenOCD需在VS Code进程PATH中。根CMake在 `project()` 前选择交叉工具链，避免误用主机编译器，ELF固定为 `build/Debug/navbot_body.elf`。

纯静态检查：

```sh
python3 tools/static_validate.py
```

推荐使用开发脚本（Debug已在本机编译通过）：

```sh
./scripts/doctor.sh
./scripts/build.sh Debug 4
./scripts/symbols.sh Debug
```

完整脚本用法见 [scripts/README.md](scripts/README.md)。格式脚本已使用clang-format23.1.3验证。原生 `cmake --preset Debug` / `cmake --build --preset Debug` 入口仍可使用；脚本额外导出BIN/HEX和大小报告。

OpenOCD使用CMSIS-DAP/DAPLink、SWD；在加载 `target/stm32f4x.cfg` **之后**设置 `adapter speed 1000`，避免目标脚本的2000kHz默认值覆盖。Cortex-Debug配置为FreeRTOS手动attach，不附带自动构建、program/erase/reset任务。启动调试仍会连接并可能暂停目标；已验证 Horco CMSIS-DAPv2 / SWD 1MHz 的连接、烧录校验与 GDB 调试；尚未单独验证外部复位线。

模板未接入串口stdio后端；生成的 `_write/_read` 弱回调不可直接作为已实现日志功能使用。需要日志时另行设计有界非阻塞通道。

## 再生成与审查

```sh
python3 tools/regenerate.py
```

macOS默认使用本机CubeMX内置JRE；其他安装位置可用 `--cubemx-dir` 指向Resources目录。脚本在临时副本中生成，检查告警类别、Core USER CODE区域和维护文件保持性，并运行静态断言；全部通过后才复制回工程。失败保留原源文件，只更新本地诊断证据。脚本不执行编译或探针命令。

- [ST UM1718：CubeMX脚本与生成](https://www.st.com/resource/en/user_manual/dm00104712-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf)
- [CMSIS-RTOS2等待与tick回绕](https://arm-software.github.io/CMSIS_6/main/RTOS2/group__CMSIS__RTOS__Wait.html)
- [FreeRTOS Cortex-M中断优先级](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)
- [OpenOCD适配器配置](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)

## ICM20948 与 OLED 采集诊断

- `BSP/C30D/c30d_soft_i2c.*`：PB10/PB11 单主开漏总线，约 3μs 半周期，SCL 等待上限 100μs，事务预算 2500μs；失败释放线路，重试前九脉冲恢复。预算在边沿等待时检查，可能增加结束条件/中断耗时，不能当作严格 WCET。
- `BSP/C30D/c30d_imu.*`：分阶段复位、100ms 等待、身份检查（0x68/0xEA）、逐项配置读回校验。±2g/±250°/s、DLPF cfg3、内部 ODR 225Hz。任务每 10ms 检查 data-ready 并连续读取 14 字节六轴和温度，读取最新值，不保存每一帧 225Hz 内部样本，采集驱动不含磁力计、DMP或车体坐标变换；陀螺仪标定和姿态解算由 Applications 层完成。
- 原始三轴数据同步转换为 m/s²、rad/s 和摄氏度。时间戳为读事务开始的 RTOS tick（1ms）；微秒周期为 DWT 读完时间之差，并非传感器内部采样时刻。DWT 的 32 位回绕通过无符号差值处理，不能用于两次观测相隔超过约 25.6 秒的计时。
- 数据有效性、身份、样本序号、最后错误、NACK/超时/卡线、无新数据/过期、重试/恢复及任务超期可通过 GDB 的 `navbot_imu_snapshot` 查看。`C30D_IMU_*` 健康码：0 初始化、1 正常、2 总线错误、3 身份错误、4 配置错误、5 过期。最后错误保留至复位，恢复后健康码回到 1。
- 总线错误立即失效，30ms 无新样本判过期；保留上一次值供诊断，但 OLED 显示 `----`。每次失败等待 1 秒再重新初始化，成功获取新样本才计恢复。`valid` 必须与时间戳新鲜度共同使用；OLED 会额外检查年龄。
- `Applications/imu_diagnostics.*`：IMU与姿态快照在同一短临界区按值发布。OLED 10Hz 显示 ROLL、PITCH、相对 YAW（度）、标定进度、成功样本间隔 DT（μs）、序号 N、年龄 AGE、健康 H、最后错误 E、标定状态 C、航向参考次数 EP、总线恢复数 R。原始数据和物理量保留在 GDB 快照中。方向仍为芯片坐标。
- OLED 使用厂家示例相同 PD11~PD14、页命令和扫描方向。引脚由 BSP 在任务运行后配置，CubeMX 的 IMU 引脚仍以输入安全启动，PB12 不启用 EXTI；OLED 没有读回/ACK，软件帧计数仅代表发送完成，实物显示需观察确认。
- `navbot_stack_free_words` 顺序为控制、通信、采集、监控，单位是 32 位字；`navbot_oled_max_us` 为包含抢占的最大已观察刷新耗时。

验证入口（主机状态机测试不连接硬件）：

```sh
./scripts/test-imu.sh
./scripts/check.sh
./scripts/format.sh --check
./scripts/build.sh Debug 4
```

实板测试需要单独明确启动，先烧录 Debug 固件并运行 `openocd -f openocd.cfg`，再在另一终端执行 `python3 tools/hardware_validate_imu.py`。脚本会暂停/恢复目标，要求电机保持断开；测试实际地址 NACK（改访无设备的 0x69）、模拟 SCL 等待超时、屏蔽 data-ready 后的过期及自动恢复，并执行 30 秒采样计数测试。模拟超时不等于已验证物理短路；没有覆盖实际断线、上电波形或所有电气故障。故障开关 `c30d_imu_test_fault` 仅 Debug 存在，默认零，脚本在 finally 中清零；Release 不含该符号与分支。

主机测试使用真实驱动和模拟寄存器总线，覆盖初始化读回、带符号转换、单位换算、NACK 退避/恢复、过期、错误身份/配置、长期运行与 tick 回绕；不验证实际总线边沿。采样、异常和电机寄存器证据位于 `build/Debug/imu-validation/`；持久测试结论见 `validation/imu-oled-bench.txt`。原始厂家 Flash 备份在 `build/Debug/hardware-validation/original-flash.bin`。

配置参考：[TDK DS-000189 v1.5](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf)，以及本机厂家 `C30D V2.0基础例程.zip` 内 `OLED.zip` / `ICM20948.zip`。仅参考寄存器和硬件映射，未复制厂家业务代码或完整字体库。

## 静态零偏标定与六轴姿态

当前用于台架功能测试，阈值和融合增益均为临时测试参数，不代表完成装车标定。C30D固定装车后，再确定芯片到车体安装矩阵、静态/温度标定与融合参数，并复测振动、线加速度和长期漂移。

`Applications/attitude.*` 为独立 C11 模块，由 sensorTask 独占。`navbot_attitude_snapshot` 可供 GDB 检查，包含有效性、标定进度/拒绝次数、零偏/方差、四元数、角度、航向参考次数和异常计数。

- 上电有效采样后等待 2 秒，再取 500 个静止样本（100Hz 下约 5 秒）。陀螺仪模长 <0.08rad/s、加速度模长为 0.90～1.10g、样本与窗口平均加速度差 <0.15m/s²；完整窗口每轴陀螺仪标准差 ≤0.003rad/s、加速度标准差 ≤0.08m/s²。运动、振动、断流或不合格窗口会重置进度；不能保证在运动中完成标定。
- 用 Welford 方法估计三轴陀螺仪均值和方差，并保存标定温度。零偏仅保留 RAM，重启重标定；没有温度补偿或后台自动零速锁定。六轴无法分辨小恒定转速与零偏，必须按 OLED 提示保持静止；2秒等待也不代表达到热稳定。
- 单姿态无法独立估计加速度三轴零偏、比例因子和安装倾角；加速度保留重力，不从每轴直接减去静止均值。初始化按平均重力计算横滚/俯仰，保留实际倾角；航向归零。
- 四元数表示芯片坐标到 Z向上的局部世界坐标，欧拉角按 ZYX（yaw-pitch-roll）右手规则，角度为度；ROLL/YAW 约为[-180,180]，PITCH 为[-90,90]。未实现芯片到车体的安装矩阵，不能直接作为 ROS base_link 姿态使用；俯仰接近±90°时欧拉角有固有奇异性，四元数仍归一化。
- 六轴非线性互补滤波：陀螺仪减静态零偏后积分四元数，加速度归一化方向与预测重力的叉积反馈（比例增益2s⁻¹，无积分项）。模长在0.85～1.15g时才使用重力反馈；模长检查不能排除所有线加速度。不会用角速度死区或静止锁航向掩盖真实漂移。
- 使用本次实际 `period_us` 积分，同时要求相邻 tick 和微秒间隔均在5～20ms。NaN/Inf、异常dt或数据过期立即使姿态失效，不积分未知间隔。若未发生设备重置，后续可信重力到来时沿用既有零偏、重建倾角并将相对航向归零。驱动恢复会重置IMU，`recovery_count` 变化时强制撤销标定有效性，重新等待2秒和收集500个静止样本，再发布姿态；`calibration_runs` 统计完成次数。每次重建 `heading_epoch`/OLED `EP` 增加，参考变化不能当作连续航向。
- 六轴缺少绝对航向约束。YAW是标定完成时起算的相对角，长期会因残余零偏、温漂而漂移；没有磁北或全局航向精度保证。横滚/俯仰同样受加速度计偏差及线加速度影响。

```sh
./scripts/test-attitude.sh
# 已烧录Debug且运行OpenOCD后，保持主板静止：
python3 tools/attitude_bench.py --seconds 60 --name static-drift
# Debug专用NACK注入：验证姿态失效、强制重新标定和航向参考重建
python3 tools/attitude_bench.py --name recovery --fault-check
```

主机测试覆盖倾斜状态标定、静止、已知三轴转动、变dt、慢速航向、方差/运动拒绝、异常数据/断流与角度/tick回绕。初版实测曾发现IMU重置后沿用旧零偏会产生约3.35°/61秒的航向漂移，故恢复后必须重标定；此历史结果保存在独立日志中，不能与最终固件漂移混淆。

实板记录工具默认只读取实际数据并断言电机寄存器与故障寄存器，等待标定后按目标tick记录指定时长，不模拟物理运动；`--fault-check` 会额外写入 Debug 地址错误注入开关并在 finally 中清除，验证失效与恢复；统计漂移须确保该窗口实际静止。GDB读取会短暂停核，DWT和RTOS tick在暂停期间停止，统计时长按目标tick计算；动态物理转动可能发生在暂停期间，故这些记录只验证响应趋势，不能评估无扰动态角度精度。证据在 `build/Debug/attitude-validation/`，持久结论见 `validation/attitude-bench.txt`。

参考：[x-io IMU/AHRS](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)、[NXP AN3461](https://www.nxp.com/docs/en/application-note/AN3461.pdf)。本模块独立实现，没有复制第三方源码或使用厂家固定5ms步长/角速度死区。
