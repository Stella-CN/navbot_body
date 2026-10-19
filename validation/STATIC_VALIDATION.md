# navbot_body 全面静态审查

日期：2026-10-19，Asia/Shanghai。范围：工程模板、生成配置、应用/BSP骨架、内核接入、CMake/VS Code/OpenOCD及生成器诊断。

**结论：已修复本轮发现的配置/结构问题，46/46项静态断言通过，隔离再生成保持性验证通过。剩余已审查的生成器告警，未发现破坏当前配置生成结果的证据。此结论只适用于本次工程与锁定版本；不等价于编译/链接或实板功能正确。**

## 1 本轮修正

| 项目 | 发现与影响 | 修正与依据 |
| --- | --- | --- |
| OpenOCD速度覆盖 | 原配置先设1000kHz再加载目标脚本；本机stm32f4x.cfg会改成2000kHz，原1MHz说明不成立 | `openocd.cfg:4`先加载目标，`:6`最后设1000；未启动OpenOCD |
| 内核启动失败 | osKernelInitialize失败未检查，osKernelStart返回后只空转 | `freertos.c`创建对象前核验osKernelReady，main在启动返回后进入故障锁定；已有任务创建空句柄检查继续保留 |
| 工具链入口 | 原根CMake依赖preset传工具链，普通`cmake -S/-B`入口可能选主机编译器 | 在project前提供Arm工具链默认值；项目显式LANGUAGES C ASM、Thumb和ELF后缀；未执行configure |
| RTC派生频率 | RTC未启用，但HSE8MHz÷2=4MHz超过数据库字段Max1MHz，导入产生invalid value | 仅把未激活的RTC/HSE分支设为÷8=1MHz；再次生成不再出现该告警；系统PLL与总线保持原值 |
| 代码职责 | 任务业务循环与寄存器故障处理混在Core和内联头中 | 参照vehicle-firmware分为Applications与BSP，Core保留生成资源及薄封装；移除旧c30d_template.h |
| 再生成可靠性 | 先覆盖正式源码后才检查，且仅看插件结果不足以判断生成完整性 | 临时副本生成→诊断分类→USER CODE/维护文件校验→静态断言→回写；未知导入/生成错误中止发布 |
| 名称一致性 | 原项目名c30d_freertos | 目录、IOC、CMake目标、工作区、ELF路径、脚本和说明统一navbot_body；旧目录已移除 |

参考工程只读取，未修改。采用其Applications/BSP/Core/Drivers/Middlewares职责划分和格式约定，未复制业务代码、硬件配置、C++算法或其他驱动。CMake使用明确源文件清单，避免递归GLOB把无关文件加入目标。

## 2 FreeRTOS审查

- 任务资源由CubeMX生成并持有；四个线程入口仅委托Applications，未重复创建线程。control/comm/sensor/monitor优先级为40/24/16/8（均小于configMAX_PRIORITIES=56）；周期为10/5/10/100ms。
- 应用栈为2048+2048+1536+1024=6656B。均有静态TCB和栈；Idle/Timer内存回调由CMSIS包装器提供，不缺少必要回调。另有Idle128字、Timer256字及8KiB heap_4，实际RAM总量仍须链接后核验。
- 当前业务/BSP没有堆分配、异步对象生命周期或共享可变业务状态；不能由此宣称整个HAL/内核不存在内存问题。未来队列、DMA缓冲和64位快照的所有权要求已写入README，但尚未实现。
- `osDelayUntil`按绝对tick调度，包装器通过无符号模差处理回绕；过期或零延迟返回错误，模板锁定而不补跑。当前空任务满足结构要求，尚无WCET/抖动证据。
- HAL初始化设置NVIC分组4；UART3与DMA逻辑优先级5，MAX_SYSCALL为5；TIM6为15且不调用RTOS。SVC/PendSV归ARM_CM4F端口，SysTick只有CMSIS包装器一个强实现；TIM6→HAL_TIM_IRQHandler→PeriodElapsedCallback→HAL_IncTick链完整。
- configENABLE_FPU=0不是当前端口禁用FPU的证据：V10.3.1头文件注明该开关用于ARMv8M；本工程ARM_CM4F端口直接启用VFP并保存S16–S31，系统启动也启用CP10/CP11，硬浮点工具链与端口一致。未无依据修改该字段。
- 初始化状态、任务创建失败、调度器异常返回、栈溢出、分配失败、configASSERT与CPU fault均进入BSP锁定。BSP不访问HAL句柄，按RCC时钟状态操作寄存器；模板没有PWM使能路径。
- 未接入硬件看门狗、完整状态机、队列或驱动采集；线程名与周期不能视作这些功能已完成。fault处理若栈已严重损坏仍无运行时保证，软件寄存器操作也不等价于硬件断能。

## 3 引脚、时钟与存储审查

- F407VET6/LQFP100：TIM10/11(PB8/PB9)、TIM9(PE5/PE6)、TIM1(PE9/11/13/14)覆盖8路PWM；生成的AF1/AF3配置与资源分配一致，PSC0/ARR16799/初始CCR0，对应168MHz÷16800=10kHz。
- TIM2(PA15/PB3)、TIM3(PB4/PB5)、TIM4(PB6/PB7)、TIM5(PA0/PA1)为TI12，AF1/AF2映射正确，32/16/16/32位ARR；没有读后清零实现，也尚未启动编码器。SWD只占PA13/14，JTAG相关引脚已供编码器使用。未发现已配置资源互相占用同一引脚。
- GPIO开关/按键/LED、IMU保留输入及PB0电池ADC符合现有新版资料。ADC仅IN8、12bit、APB2÷4=21MHz、480周期采样，未启用采集。电机GPIO下拉、IMU不驱动，未擅自引入I²C或虚构传感器数据。
- HSE8MHz→PLL M8/N336/P2得到168MHz，Q7得到48MHz；APB1÷4=42MHz、APB2÷2=84MHz，定时器分别84/168MHz，Flash等待5周期。8MHz来自新版厂家源码证据，实物仍需确认。
- USART3 PD8/PD9 460800/8N1；USART1 PA9/PA10 115200/8N1；USART3 DMA1 RX Stream1 / TX Stream3 Channel4、字节宽、内存递增、RX循环/TX普通、句柄链接与IRQ调用逐项核对。USART3理想量化波特率约461538（+0.16%），不能替代实际串口压力测试。
- 链接脚本Flash512KiB、普通SRAM128KiB、独立CCM64KiB，与VET6一致；任务栈/堆默认.bss在普通SRAM。CCM不得作为DMA缓冲。Flash起点为0x08000000；未适配厂家Bootloader偏移和向量表，后续下载前必须决定保留策略。
- 电机A/B/C/D到实际轮位、方向极性、有效计数、最大脉冲率与过滤15的关系、PI、轮距、电池阈值、真实停车输出均未验证。

## 4 CMake / VS Code / OpenOCD审查

- 根CMake在语言探测前选择交叉工具链，显式C11/C ASM，无业务C++需求；generated HAL与FreeRTOS对象使用相同CPU/FPU ABI和Thumb模式。与生成文件保持相同的plain target_link_libraries签名，避免CMake签名混用。
- BSP/Application包含路径通过stm32cubemx接口传播，FreeRTOSConfig中的BSP panic声明对FreeRTOS对象可见；BSP实现明确加入最终ELF，不会仅给主程序设置包含路径而遗漏内核对象。
- 读取并检查所有生成CMake源文件/包含路径存在，链接脚本和startup路径一致；显式项目源清单只含Applications/app_tasks.c、BSP/C30D/c30d_safety.c。未运行CMakeLists配置或GCC，不能证明最终符号解析/工具链版本兼容。
- `cmake --list-presets=all`成功列出Debug/Release配置和构建预设，该命令只解析预设，不执行configure/try_compile。VS Code关闭configureOnOpen，JSON可解析，无自动preLaunchTask。
- 工作区/IOC/CMake目标/Debug ELF均命名navbot_body。OpenOCD使用本机存在的cmsis-dap.cfg及stm32f4x.cfg，SWD、最终1000kHz；Cortex-Debug attach schema支持rtos=FreeRTOS，未启动服务器、探测探针或发送reset/program/erase。
- 生成的newlib `_write/_read`后端仍为弱回调，模板未使用stdio；不可将其当作已接入日志功能。未来日志必须另行提供有界后端。

## 5 生成器告警影响判定

没有把“插件Error:false”或进程退出0单独当作正确性证明。此次分类、生成源码、受控实验和依赖来源共同构成证据。

| 诊断 | 本次数量 | 影响判断与证据 |
| --- | --- | --- |
| RTC `RTCHSEDivFreq_Value` invalid | 0（修复后） | 已消除；未激活RTC分支÷8，系统PLL不变 |
| USART3 DMA参数“currently not set” | 20 | 当前字段均与IOC一致；临时副本将RX/TX优先级LOW→HIGH，告警仍在但两路生成结果均为HIGH，说明此次优先级设置未被忽略；正式工程保持LOW。见DMA_IMPORT_EXPERIMENT.json。分类名dma_import_order为导入阶段现象描述，不宣称已定位CubeMX内部根因 |
| RTOS API schema加载失败 | 1 | 生成器schema兼容问题仍存在；四任务属性、栈、入口、包装器、hooks、内核flags、再生成保持性已核对。未观察到当前模板输出缺失，不承诺未来所有RTOS选项均可正确生成 |
| FreeRTOS新参数缺失 | 18 | DB引用configHEAP_CLEAR_MEMORY_ON_FREE/INCLUDE_xTaskDelayUntil；锁定的V10.3.1使用INCLUDE_vTaskDelayUntil且heap_4不读取前者，当前调用链存在。不能据此升级或混入其他版本内核 |
| 未选扩展包PDSC不支持 | 4 | IOC ThirdPartyNb=0，使用本地CubeF4依赖；121个Drivers/Middlewares文件逐一与V1.28.3比对，零差异，不从这些X-CUBE包生成依赖 |
| RIF虚拟引脚不可用于此MCU | 1 | 最终IOC与源代码无RIF，F407未配置该功能；该告警不对应实际占用的物理引脚。内部来源未进一步断言 |
| 空RTOS可选表 | 1 | 没有定义队列/信号量/用户定时器；四任务表非空且全部生成。未发现当前资源遗漏 |
| 数据库/扩展包/UI环境WARN | 138 | 完整日志保留。包括未选包模式、数据库路径、重复XSPI组件和锁等待；当前选用F4资源均已检查，不将这些警告当作固件运行证据 |

`generator-diagnostics.json`的unresolved_errors为空，意为本轮未出现未分类的导入/生成错误，**不是原始日志无ERROR**。已审查类别仍有日志ERROR/PDSC消息；没有修改本机CubeMX安装、包数据库或全局设置来掩盖告警。

## 6 可复核证据与限制

- `STATIC_CHECKS.txt`：46项断言的完整输出。
- `regeneration-check.json`：隔离生成通过；22个Core文件的USER CODE区域和16个维护文件保持，未调用编译或探针。
- `DEPENDENCY_ORIGIN.json`：121个第三方依赖文件与锁定包一致。
- `DMA_IMPORT_EXPERIMENT.json`：仅在临时副本改变优先级，正式源码未受实验变更影响。
- `REVIEWED_FILES.json`：关键审查文件SHA-256摘要。
- `cubemx-generation.log`：最终原始日志，本地保留并按仓库规则忽略。

本轮仅执行Python静态检查、CubeMX代码生成和CMake预设列表读取；未执行configure、try_compile、GCC/链接器、OpenOCD、下载或实机测试。源树无build目录和ELF/HEX/BIN/目标文件。剩余编译、内存实际占用、调度实时性、电气行为、探针连通与Bootloader策略不属于此次通过结论。

参考：[ST STM32F407VE资料](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ve.html)、[CMSIS osDelayUntil](https://arm-software.github.io/CMSIS_6/main/RTOS2/group__CMSIS__RTOS__Wait.html)、[FreeRTOS Cortex-M端口](https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/ARM-Cortex/RTOS-Cortex-M3-M4)、[OpenOCD适配器配置](https://openocd.org/doc/html/Debug-Adapter-Configuration.html)。审查以本机锁定源码和实际生成结果为主，官网资料用于核对接口语义。
