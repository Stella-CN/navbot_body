# C30D 电机基础模块

## 范围与技术栈

Applications 全部使用 C++17；BSP、CubeMX、HAL、FreeRTOS 仍是 C11。原应用 `.h` 保留 `extern "C"` 边界，现有 C 任务入口、HAL UART 回调和协议测试不改变 ABI。C++ 禁用异常、RTTI、线程安全局部静态守卫；电机对象不分配堆内存，构造不访问外设。

这里只提供电机信息、显式使能、PWM、滑行、制动与编码器反馈。不提供目标速度、PID、运动学、轨迹、通讯运动指令、电流/力矩估算。原串口运动指令仍拒绝，轮速协议占位字段未接入本模块。

## 继承与所有权

```text
Motor（抽象基类，通用状态/参数/反馈）
└── C30dMotor（AT8236 + 定时器适配）
    ├── FrontLeftMotor   C / 左前
    ├── FrontRightMotor  B / 右前
    ├── RearLeftMotor    D / 左后
    └── RearRightMotor   A / 右后
```

`WheelMotors::instance()` 拥有唯一一组静态对象；具体轮类构造函数只对该容器开放。禁止复制/移动；基类析构函数为 protected、非虚，不允许通过基类指针删除。引用不转移所有权，快照按值返回。只有 controlTask 可调用这些接口，包括读取；其他任务后续应通过队列提交请求，通过受保护的值拷贝获取快照，不允许 ISR 直接调用。

`MotorInfo` 包含轮位、接口、型号、额定电压、减速比、编码器 PPR、倍频、计数器宽度及计数校准标记。MG513P10-12V GMR 参数来自用户提供资料：12 V、1:10、500 PPR。暂以 `500 × 4 × 10 = 20000` 计数/输出轴圈换算 RPM；`counts_calibrated=false`，不表示外部转速计或手动整圈标定已完成。

## 硬件与方向

| 轮位 | 接口 | PWM 第一/第二通道 | 编码器 | 位宽 | 默认驱动极性 | 默认编码器极性 |
| --- | --- | --- | --- | --- | --- | --- |
| 右后 | A | TIM10_CH1 / TIM11_CH1，PB8/PB9 | TIM2，PA15/PB3 | 32 | +1 | -1 |
| 右前 | B | TIM9_CH1 / CH2，PE5/PE6 | TIM3，PB4/PB5 | 16 | +1 | +1 |
| 左前 | C | TIM1_CH1 / CH2，PE9/PE11 | TIM4，PB6/PB7 | 16 | +1 | +1 |
| 左后 | D | TIM1_CH3 / CH4，PE13/PE14 | TIM5，PA0/PA1 | 32 | +1 | -1 |

“第一通道”按上表的物理定时器通道定义，不采用厂家 PWMC1/PWMD1 的编号推断方向。正 PWM 为第一通道常高、第二通道高电平占比 `1-duty`；负 PWM 对调。10 kHz、ARR=16799，CCR=16800 表示真正的常高。基础 PWM 使用 AT8236 慢衰减。

2026-11-07 临时台架测试中，正电气 PWM 的原始编码器符号为 A负/B正/C正/D负，故默认对 A/D 编码器取反，以使正驱动对应正转速反馈。**该约定不是车体前进方向**。前进方向实测后，在 disabled 状态下分别配置驱动/编码器极性，并重新初始化。两种极性是独立参数，不能仅改驱动极性而忽略反馈符号。

BSP 在更新一对通道前短暂关闭该对输出，再写入 CCR 并恢复；只清除本对的 OC preload，避免等待下一个周期时输出旧值。C/D 不修改共享 TIM1 的 ARR/CNT、不发送 UG、不关闭另一轮通道；仅当 TIM1 所有轮通道禁用时清除 MOE。中断屏蔽区仅包含有界寄存器操作，恢复调用前的 PRIMASK。

## 基础 API

| 方法 | 契约 |
| --- | --- |
| `initialize(now_ms)` | 必须在 CubeMX 外设初始化之后；禁用本路输出、验证配置、启动编码器，清零软件累计计数并建立原始计数基线。硬件 CNT 不清零。 |
| `updateFeedback(now_ms)` | 正常每 10 ms 调用；返回采样成功状态，使用实际毫秒间隔计算计数/秒和 RPM。 |
| `enable()` | 必须已初始化、有有效反馈且无锁存错误；使能逻辑状态，但不输出 PWM。 |
| `setDuty(permille)` | 有符号千分比；默认范围 ±200（20%）。越界、未使能或硬件错误时停止并锁存错误，不静默截断。零值等同滑行。 |
| `coast()` | 本路双输入低/输出禁用，机械自由停止；保留已使能状态。 |
| `brake()` | 需要显式使能；双输入常高，AT8236 电气制动，保持 Braking 状态。 |
| `disable()` | 本路滑行并撤销使能，不清除锁存错误。 |
| `configure(limit, drive_sign, encoder_sign)` | 仅非使能状态允许，限幅为 1..1000，极性为 ±1；修改后必须重新 initialize。 |
| `info()/feedback()/state()/error()/duty()` | 按值读取信息或状态。反馈含原始 CNT、逻辑增量/累计计数、计数/秒、RPM、时间戳、有效性。 |
| `WheelMotors::disableAll()` | 对四轮逐个撤销使能。中断/CPU 故障仍走独立的 `C30D_ForceMotorIdle()`。 |

错误后必须 `initialize()` 并取得新采样才能重新使能；正常 `disable()` 后可显式重新使能。`enable()` 不代表机械解锁或电源接通。PD3 使能开关是软件输入，本模块不假定其有效电平、不将其实现成硬件急停。

调用示意（只能放在控制任务里，当前默认应用未加入这些运动调用）：

```cpp
auto &wheels = navbot::WheelMotors::instance();
// 初始化阶段一次：wheels.initialize(now_ms);
// 每个10ms周期先完成：wheels.updateFeedback(now_ms);
auto &motor = wheels.frontLeft();
if (explicit_enable_request && motor.enable()) {
    // 后续 PID 也只输出占空比给这个接口，PID 状态留在外部。
    if (!motor.setDuty(100)) {
        wheels.disableAll();
    }
}
const auto measured = motor.feedback();
// 明确的停止请求：motor.disable();
```

`explicit_enable_request` 和 `now_ms` 由未来控制层提供。不得每周期重复初始化、自动清错或自动使能；读取反馈前必须检查 `valid` 和时间戳。所有方法本身不延时、不等待电机到速/停稳。

## 计数、采样与边界

TIM2/5 使用 32 位计数，TIM3/4 使用 16 位计数。模差分处理正反回绕；累计值是每对象独立的 int64，异常间隔、半量程歧义和算术溢出均失败并停止。初始化后第一份反馈无效，首次 1..20 ms 样本才有效。tick 使用 uint32 差分处理回绕；0 ms 或超过 20 ms 的间隔使速度无效，不用旧值伪造本周期速度。

在用户给出的 1100±132 RPM 和 20000 计数/圈条件下，20 ms 内低于 16 位半量程。若更换电机/倍频、提高转速，应重新核算采样上限；软件无法识别采样间隔内已发生的整圈计数器回绕。原始信号毛刺和机械振动仍可能造成少量计数。

默认 controlTask 在初始化时启动四路编码器，按 100 Hz 更新反馈；无 PWM 使能入口调用。当前 RTOS 时基要求 1000 Hz，启动时检查。任一路采样失败先禁用四路，再进入现有故障锁定。

本模块不内置命令超时、IWDG、斜坡、电流闭环或机械停稳判定。运行中的调用者必须持续执行控制周期并处理故障，后续整车控制层应实现看门狗/失联停机；单靠基础类不能覆盖任务完全卡死。直接反转会立即改变电气方向，外部控制层负责先减速/停止。滑行停止不保证瞬间停轮；制动也不等于硬件断能。

## 验证与证据

```sh
./scripts/test-motor.sh
./scripts/test-link.sh
./scripts/test-attitude.sh
./scripts/test-imu.sh
./scripts/check.sh
./scripts/format.sh --check
./scripts/build.sh Debug 4
./scripts/build.sh Release 4
```

电机测试编译真实 C++ 模块和 C BSP，使用独立寄存器模拟；覆盖生命周期、越界停机、极性、C ABI、编码器/时间回绕、异常采样、四轮映射、C/D共享 TIM1 隔离及中断状态恢复。测试不模拟真实桥臂时序、PWM波形、电流、制动力或负载。

默认 UBSan；支持 `SANITIZERS=address,undefined ./scripts/test-motor.sh`。本机 Apple Clang 17 的 ASan 在最小独立示例启动阶段也挂起，因此本轮不能报告 ASan 通过。此模块没有动态分配；最终 ARM ELF 不应引入 `operator new/delete`、异常处理或局部静态守卫。

原台架数据位于本地 `build/motor-bench-20261107/summary.json`，对应此前临时测试固件，仅用于引脚/极性与慢衰减方式依据，不能视为本次新模块的实板验证。本次不烧录、不启动电机。

参考资料：

- 用户提供的 `9.原理图/C30D 原理图和资源分配表/STM32F407VET6控制器原理图（C30D_V2.2）.pdf` 第3页，以及 `STM32F407VET6(C30D-V2.1_V2.2)主板资源分配说明.pdf`。前者核对 AT8236、引脚和接口，后者核对定时器资源。
- `/Users/lvjiaqing/ForWork/vehicle-firmware/Applications/motor.hpp`、`motor.cpp`：借鉴电机对象封装与参数/反馈分离；该项目为 ODrive，不移植 CAN/力矩/PID 接口。其方法内部共享 static 状态也不沿用。
- [ST RM0090](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)：定时器 PWM、预装载、编码器接口与计数范围。
- [AT8236 原厂手册（分销商镜像）](https://atta.szlcsc.com/upload/public/pdf/source/20210923/1552D95CBAF8BE780D06AE79E7783380.pdf)：PWM输入、双高制动、双低滑行及慢衰减。
- [C++ Core Guidelines C.35 / C.67](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines)：基类析构访问规则与多态对象复制约束。

## 后续实板验证（2026-11-07）

用户随后授权架空实测，使用独立测试副本调用本模块的真实 API，完成四轮各三项低占空比测试。最终已烧录不含测试入口的新固件并确认输出禁用。B滑行与C制动释放后存在少量残余计数，未将其计作严格零计数通过；完整条件、结果与局限见 [实板记录](../validation/motor-hardware.txt)。上述软件验证阶段未烧录的记录属于先前阶段。
