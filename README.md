# navbot_body

NavBot C30D 小脑固件模板，STM32F407VET6 + HAL + FreeRTOS + C11。

当前仅完成模板生成与静态审查；未配置构建、编译、链接、下载或连接探针。已用 `cmake --list-presets=all` 只读解析预设。电机闭环、协议、IMU、采样和看门狗尚未实现，输出保持禁用。

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
| IMU预留 | PB10/PB11/PB12保持输入，不驱动总线 |
| 调试 | PA13/PA14 SWD；编码器使用原JTAG复用引脚 |

C/D方向不能按通道顺序推断。A/B/C/D到实际车轮的映射、编码器极性和每圈计数、轮径、有效轮距、PI、电池阈值均待实测，不填入示例值。过滤15须结合最高编码器脉冲频率核对。

## FreeRTOS职责与边界

| 任务 | CMSIS优先级 | 周期 | 静态栈 | 后续唯一所有权 |
| --- | --- | --- | --- | --- |
| controlTask | High / 40 | 10ms | 2048B | 编码器模差分、四路PI、运动状态、PWM |
| commTask | Normal / 24 | 5ms | 2048B | 串口收发缓冲、校验/会话、固定容量消息 |
| sensorTask | BelowNormal / 16 | 10ms | 1536B | 有界IMU/ADC采集、样本时间与有效性 |
| monitorTask | Low / 8 | 100ms | 1024B | 新鲜度、栈水位、诊断及看门狗策略 |

四个应用任务均静态分配，栈合计6656B；CMSIS包装器提供Idle/Timer静态内存回调。内核仍启用heap_4（8192B）与定时器服务，不能称为全静态内核。当前维护代码不分配堆内存，也不跨任务共享可变业务数据。

任务采用绝对周期等待，tick回绕由CMSIS包装器处理；截止时间过期（包含正好等于当前tick）返回错误并锁定。尚未测量WCET、抖动或栈水位。不能将这些周期声明等同于已通过实时性验证。

内核初始化后检查 `osKernelReady`，任务创建检查空句柄，调度器启动若返回即锁定。故障处理不使用HAL句柄、RTOS或阻塞I/O，先关闭IRQ，再按外设时钟状态清零CCR、关闭CCER/MOE。模板没有使能API，控制任务持续保持禁用；软件输出禁止不等于已验证的驱动电平、停车能力或硬件断能。

UART/DMA中断逻辑优先级5，HAL TIM6与RTOS tick为15，NVIC分组4。数值小于5的ISR不得调用FreeRTOS API；TIM6不调用RTOS。SVC/PendSV归内核，SysTick归CMSIS包装器，TIM6归HAL。

后续使用值拷贝固定容量队列传递命令/快照：通信任务完成验证后交给控制任务，控制任务独占PI和运动状态；DMA缓冲在完成前归驱动，ISR只做有界通知；64位快照需同步。队列、协议、状态机、闭环、微秒时间戳、采样和看门狗均尚未实现。

## VS Code与OpenOCD

打开 `navbot_body.code-workspace` 或此目录。使用 CMake Tools、C/C++，调试可选 Cortex-Debug；不依赖STM32官方插件。未安装扩展。打开工作区不会自动配置或构建。

`arm-none-eabi-gcc`、`arm-none-eabi-gdb`、CMake、Ninja及OpenOCD需在VS Code进程PATH中。根CMake在 `project()` 前选择交叉工具链，避免误用主机编译器，ELF固定为 `build/Debug/navbot_body.elf`。

纯静态检查：

```sh
python3 tools/static_validate.py
```

后续手动构建命令（当前未执行）：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

OpenOCD使用CMSIS-DAP/DAPLink、SWD；在加载 `target/stm32f4x.cfg` **之后**设置 `adapter speed 1000`，避免目标脚本的2000kHz默认值覆盖。Cortex-Debug配置为FreeRTOS手动attach，不附带自动构建、program/erase/reset任务。启动调试仍会连接并可能暂停目标；探针、复位线和调试会话均未实测。

模板未接入串口stdio后端；生成的 `_write/_read` 弱回调不可直接作为已实现日志功能使用。需要日志时另行设计有界非阻塞通道。

## 再生成与审查

```sh
python3 tools/regenerate.py
```

macOS默认使用本机CubeMX内置JRE；其他安装位置可用 `--cubemx-dir` 指向Resources目录。脚本在临时副本中生成，检查告警类别、Core USER CODE区域和维护文件保持性，并运行静态断言；全部通过后才复制回工程。失败保留原源文件，只更新本地诊断证据。脚本不执行编译或探针命令。

当前剩余生成器告警及影响判断见[全面静态审查](validation/STATIC_VALIDATION.md)，分类结果见 `validation/generator-diagnostics.json`。原始 `.log` 按仓库规则保留在本地，不纳入版本控制。今后出现未识别的导入/生成错误，脚本会停止发布，不能仅凭两项插件返回false宣称工程正确。

- [ST UM1718：CubeMX脚本与生成](https://www.st.com/resource/en/user_manual/dm00104712-stm32cubemx-for-stm32-configuration-and-initialization-c-code-generation-stmicroelectronics.pdf)
- [CMSIS-RTOS2等待与tick回绕](https://arm-software.github.io/CMSIS_6/main/RTOS2/group__CMSIS__RTOS__Wait.html)
- [FreeRTOS Cortex-M中断优先级](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)
- [OpenOCD适配器配置](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)
