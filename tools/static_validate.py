#!/usr/bin/env python3
"""Read-only firmware template checks. Never invokes a compiler or probe."""
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
checks = []


def require(condition, description):
    checks.append((bool(condition), description))


def read(name):
    return (ROOT / name).read_text()


def function(text, name):
    match = re.search(r'void\s+' + re.escape(name) + r'\([^)]*\)\s*\{', text)
    if not match:
        return ''
    start = match.end()
    level = 1
    end = start
    while level and end < len(text):
        level += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end - 1]


def main():
    ioc = dict(line.split('=', 1) for line in read('navbot_body.ioc').splitlines() if '=' in line)
    require(ioc.get('Mcu.CPN') == 'STM32F407VET6', 'STM32F407VET6 / LQFP100 target')
    pinmap = {'PB8': 'S_TIM10_CH1', 'PB9': 'S_TIM11_CH1', 'PE5': 'S_TIM9_CH1',
              'PE6': 'S_TIM9_CH2', 'PE9': 'S_TIM1_CH1', 'PE11': 'S_TIM1_CH2',
              'PE13': 'S_TIM1_CH3', 'PE14': 'S_TIM1_CH4', 'PA15': 'S_TIM2_CH1_ETR',
              'PB3': 'S_TIM2_CH2', 'PB4': 'S_TIM3_CH1', 'PB5': 'S_TIM3_CH2',
              'PB6': 'S_TIM4_CH1', 'PB7': 'S_TIM4_CH2', 'PA0-WKUP': 'S_TIM5_CH1',
              'PA1': 'S_TIM5_CH2', 'PD8': 'USART3_TX', 'PD9': 'USART3_RX',
              'PA9': 'USART1_TX', 'PA10': 'USART1_RX', 'PB0': 'ADCx_IN8',
              'PA13': 'SYS_JTMS-SWDIO', 'PA14': 'SYS_JTCK-SWCLK'}
    require(all(ioc.get(pin + '.Signal') == sig for pin, sig in pinmap.items()),
            '23 motor/encoder/UART/ADC/SWD pin assignments match board evidence')
    pins = [ioc.get(f'Mcu.Pin{i}') for i in range(int(ioc['Mcu.PinsNb']))]
    require(None not in pins and len(pins) == len(set(pins)), 'pin enumeration complete and unique')
    for pin in ('PB10', 'PB11', 'PB12'):
        require(ioc.get(pin + '.Signal') == 'GPIO_Input', f'{pin} reserved for future IMU driver')
    main_c, tim = read('Core/Src/main.c'), read('Core/Src/tim.c')
    require(all(s in main_c for s in ('PLLM = 8;', 'PLLN = 336;', 'PLLP = RCC_PLLP_DIV2;',
                                      'PLLQ = 7;', 'APB1CLKDivider = RCC_HCLK_DIV4;',
                                      'APB2CLKDivider = RCC_HCLK_DIV2;'))
            and re.search(r'#define HSE_VALUE\s+8000000U', read('Core/Inc/stm32f4xx_hal_conf.h')),
            '8MHz HSE -> 168MHz SYSCLK, APB1 42MHz / APB2 84MHz, PLLQ 48MHz')
    require(all('Init.Period = 16800-1;' in function(tim, 'MX_' + t + '_Init')
                and 'Init.Prescaler = 0;' in function(tim, 'MX_' + t + '_Init')
                and 'sConfigOC.Pulse = 0;' in function(tim, 'MX_' + t + '_Init')
                for t in ('TIM1', 'TIM9', 'TIM10', 'TIM11')), 'all eight PWM channels: 10kHz, zero initial duty')
    require(all('Init.Period = ' + value + ';' in function(tim, 'MX_' + t + '_Init')
                and 'TIM_ENCODERMODE_TI12' in function(tim, 'MX_' + t + '_Init')
                for t, value in [('TIM2', '4294967295'), ('TIM3', '65535'),
                                 ('TIM4', '65535'), ('TIM5', '4294967295')]),
            'quadrature encoder counters use native 32/16/16/32-bit ranges')
    app = '\n'.join(p.read_text() for directory in ('Core', 'Applications', 'BSP') for p in (ROOT / directory).rglob('*.c'))
    require(not re.search(r'HAL_TIM_(?:PWM|Encoder)_Start\s*\(', app)
            and not re.search(r'HAL_UART(?:Ex)?_(?:Receive|Transmit)\w*\s*\(', app),
            'template never starts motors, encoders, or UART transfers')
    irq = read('Core/Src/stm32f4xx_it.c')
    timebase = read('Core/Src/stm32f4xx_hal_timebase_tim.c')
    wrapper = read('Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c')
    require('HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn)' in timebase
            and 'HAL_TIM_IRQHandler(&htim6)' in function(irq, 'TIM6_DAC_IRQHandler')
            and 'HAL_IncTick();' in function(main_c, 'HAL_TIM_PeriodElapsedCallback')
            and 'void SysTick_Handler' not in irq and 'xPortSysTickHandler();' in wrapper,
            'HAL tick -> TIM6_DAC; single RTOS SysTick handler -> CMSIS wrapper')
    uart = read('Core/Src/usart.c')
    require('huart3.Init.BaudRate = 460800;' in uart
            and 'hdma_usart3_rx.Instance = DMA1_Stream1;' in uart
            and 'hdma_usart3_tx.Instance = DMA1_Stream3;' in uart
            and uart.count('Init.Channel = DMA_CHANNEL_4;') == 2
            and 'hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;' in uart,
            'USART3 460800, DMA1 RX stream1 / TX stream3 channel4, circular RX')
    require(all(f'HAL_NVIC_SetPriority({name}, 5, 0)' in app for name in
                ('USART3_IRQn', 'DMA1_Stream1_IRQn', 'DMA1_Stream3_IRQn'))
            and ioc.get('NVIC.PriorityGroup') == 'NVIC_PRIORITYGROUP_4',
            'RTOS-capable UART/DMA interrupts at logical priority5, group4')
    rtos, cfg = read('Core/Src/freertos.c'), read('Core/Inc/FreeRTOSConfig.h')
    application = read('Applications/app_tasks.c')
    require(all(f'.cb_mem = &{name}Tcb' in rtos and f'.stack_mem = &{name}Stack[0]' in rtos
                for name in ('control', 'comm', 'sensor', 'monitor')),
            'all four application tasks own static TCBs and stacks')
    require(all(f'osKernelGetTickFreq() / {hz}U' in function(application, 'Navbot_' + role + 'Task')
                and 'osDelayUntil(deadline)' in function(application, 'Navbot_' + role + 'Task')
                for role, hz in [('Control', 100), ('Comm', 200), ('Sensor', 100), ('Monitor', 10)]),
            'absolute task periods: control100 / comm200 / sensor100 / monitor10 Hz')
    require(bool(re.search(r'configCHECK_FOR_STACK_OVERFLOW\s+2', cfg))
            and bool(re.search(r'configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY\s+5', cfg))
            and all('C30D_TemplatePanic();' in function(rtos, f) for f in
                    ('vApplicationStackOverflowHook', 'vApplicationMallocFailedHook')),
            'stack/heap hooks and kernel assert latch motor inhibition')
    require(all('C30D_TemplatePanic();' in function(irq, f) for f in
                ('HardFault_Handler', 'MemManage_Handler', 'BusFault_Handler', 'UsageFault_Handler')),
            'CPU fault handlers latch motor inhibition')
    linker = read('STM32F407XX_FLASH.ld')
    require(all(s in linker for s in ('ORIGIN = 0x20000000, LENGTH = 128K',
                                     'ORIGIN = 0x10000000, LENGTH = 64K',
                                     'ORIGIN = 0x8000000, LENGTH = 512K')),
            'linker: 512KiB Flash, 128KiB DMA-visible SRAM, separate 64KiB CCM')
    missing = []
    for cmake in ROOT.rglob('CMakeLists.txt'):
        for ref in re.findall(r'\$\{CMAKE_CURRENT_SOURCE_DIR\}/([^\s)]+)', cmake.read_text()):
            if not (cmake.parent / ref.rstrip(chr(34))).exists():
                missing.append(ref)
    require(not missing, 'all generated CMake source/include references exist: ' + str(missing))
    for path in list((ROOT / '.vscode').glob('*.json')) + [ROOT / 'CMakePresets.json', ROOT / 'navbot_body.code-workspace']:
        json.loads(path.read_text())
    require(True, 'VS Code / CMake / workspace JSON parses')
    require('cmake.configureOnOpen": false' in read('.vscode/settings.json')
            and 'cmsis-dap.cfg' in read('openocd.cfg')
            and '"request": "attach"' in read('.vscode/launch.json'),
            'no configure-on-open; manual OpenOCD CMSIS-DAP attach configuration')
    artifacts = [p for p in ROOT.rglob('*') if p.suffix in ('.elf', '.o', '.bin', '.hex')]
    require(not artifacts and not (ROOT / 'build').exists(), 'no firmware build artifacts or build directory')
    require('project(navbot_body LANGUAGES C ASM)' in read('CMakeLists.txt')
            and ioc.get('ProjectManager.ProjectName') == 'navbot_body'
            and ioc.get('ProjectManager.ProjectFileName') == 'navbot_body.ioc',
            'project/IOC/workspace identifiers consistently use navbot_body')
    require(not (ROOT.parent / 'c30d_freertos').exists(), 'obsolete firmware directory removed')
    cmake_root = read('CMakeLists.txt')
    require(cmake_root.index('set(CMAKE_TOOLCHAIN_FILE') < cmake_root.index('\nproject(')
            and 'CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY' in read('cmake/gcc-arm-none-eabi.cmake'),
            'cross toolchain selected before language detection; compiler probes limited to static libraries')
    require('Applications/app_tasks.c' in cmake_root and 'BSP/C30D/c30d_safety.c' in cmake_root
            and 'GLOB' not in cmake_root and 'target_include_directories(stm32cubemx INTERFACE' in cmake_root,
            'explicit Applications/BSP source list; BSP declarations propagate to RTOS/HAL objects')
    require('target_link_libraries(${PROJECT_NAME} PRIVATE' not in cmake_root,
            'root/generated executable use consistent plain target_link_libraries signature')
    require('osKernelGetState() != osKernelReady' in function(rtos, 'MX_FREERTOS_Init')
            and main_c.index('C30D_TemplatePanic();', main_c.index('osKernelStart();')) > main_c.index('osKernelStart();'),
            'kernel initialization state and unexpected scheduler return are guarded')
    require(all('Navbot_' + role + 'Task(argument);' in function(rtos, 'Start' + role + 'Task')
                for role in ('Control', 'Comm', 'Sensor', 'Monitor')),
            'generated task entry points delegate once to the Applications layer')
    require(not (ROOT / 'Core/Inc/c30d_template.h').exists()
            and 'TIM1->BDTR' in read('BSP/C30D/c30d_safety.c')
            and '#include "c30d_safety.h"' in cfg,
            'register-level motor inhibition belongs to BSP; no obsolete inline safety implementation')
    port = read('Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c')
    require('vPortEnableVFP();' in port and 'vstmdbeq r0!, {s16-s31}' in port,
            'hard-float CM4F port enables FPU and preserves floating-point task context')
    require('vApplicationGetIdleTaskMemory' in wrapper and 'vApplicationGetTimerTaskMemory' in wrapper
            and 'static StackType_t' in wrapper,
            'CMSIS wrapper supplies idle/timer static allocation callbacks')
    require('RCC_RTCCLKSOURCE_HSE_DIV8' == ioc.get('RCC.RCC_RTC_Clock_Source_FROM_HSE')
            and ioc.get('RCC.RTCHSEDivFreq_Value') == '1000000'
            and not any(value == 'RTC' for key, value in ioc.items() if re.fullmatch(r'Mcu.IP[0-9]+', key)),
            'inactive RTC HSE divider is legal (8MHz/8=1MHz); RTC remains disabled')
    ocd = read('openocd.cfg')
    require(ocd.index('adapter speed 1000') > ocd.index('source [find target/stm32f4x.cfg]'),
            '1MHz OpenOCD adapter override applied after target script defaults')
    launch = json.loads(read('.vscode/launch.json'))['configurations'][0]
    require(launch.get('rtos') == 'FreeRTOS' and launch.get('request') == 'attach'
            and launch.get('executable') == '${workspaceFolder}/build/Debug/navbot_body.elf'
            and 'preLaunchTask' not in launch,
            'manual FreeRTOS OpenOCD attach references the renamed ELF; no automatic build/flash')
    require('RCC_APB2ENR_TIM1EN' in read('BSP/C30D/c30d_safety.c')
            and '__disable_irq();' in read('BSP/C30D/c30d_safety.c'),
            'fault inhibition checks peripheral clocks and latches with interrupts disabled')
    require('HAL_InitTick(TICK_INT_PRIORITY)' in read('Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c')
            and re.search(r'TICK_INT_PRIORITY\s+15U', read('Core/Inc/stm32f4xx_hal_conf.h')),
            'HAL initialization installs dedicated TIM6 tick at logical priority15')
    adc = read('Core/Src/adc.c')
    require(all(text in adc for text in ('ADC_CLOCK_SYNC_PCLK_DIV4', 'ADC_RESOLUTION_12B',
                                         'ADC_CHANNEL_8', 'ADC_SAMPLETIME_480CYCLES',
                                         'hadc1.Init.NbrOfConversion = 1;')),
            'battery ADC: PB0/channel8 only, 21MHz ADC clock, 12-bit, 480-cycle sample')
    require(all(ioc.get(pin + '.GPIO_PuPd') == 'GPIO_PULLDOWN'
                for pin in ('PB8', 'PB9', 'PE5', 'PE6', 'PE9', 'PE11', 'PE13', 'PE14')),
            'PWM pins retain pulldowns while outputs remain unstarted')
    require(all(text in tim for text in ('GPIO_AF1_TIM1', 'GPIO_AF1_TIM2', 'GPIO_AF2_TIM3',
                                         'GPIO_AF2_TIM4', 'GPIO_AF2_TIM5', 'GPIO_AF3_TIM9',
                                         'GPIO_AF3_TIM10', 'GPIO_AF3_TIM11')),
            'generated PWM and encoder GPIO alternate function families match F407 mappings')
    require(all('TIM_CHANNEL_' + str(channel) in function(tim, 'MX_TIM1_Init') for channel in range(1, 5))
            and all('TIM_CHANNEL_' + str(channel) in function(tim, 'MX_TIM9_Init') for channel in range(1, 3)),
            'all four TIM1 and both TIM9 PWM channels are initialized')
    require(all('hdma_usart3_' + direction + '.Init.' + field + ' = ' + value + ';' in uart
                for direction in ('rx', 'tx') for field, value in
                [('MemInc', 'DMA_MINC_ENABLE'), ('PeriphInc', 'DMA_PINC_DISABLE'),
                 ('MemDataAlignment', 'DMA_MDATAALIGN_BYTE'), ('PeriphDataAlignment', 'DMA_PDATAALIGN_BYTE'),
                 ('Priority', 'DMA_PRIORITY_LOW'), ('FIFOMode', 'DMA_FIFOMODE_DISABLE')])
            and 'hdma_usart3_tx.Init.Mode = DMA_NORMAL;' in uart
            and '__HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx)' in uart
            and '__HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx)' in uart,
            'DMA warnings do not alter widths, increment, priority, FIFO, TX mode or handle linkage')
    require('add_compile_options(-mthumb)' in cmake_root
            and 'target_link_options(${PROJECT_NAME} PRIVATE -mthumb)' in cmake_root
            and 'PROPERTIES SUFFIX ".elf"' in cmake_root,
            'Thumb mode and debug ELF suffix explicitly match Cortex-M4 and launch configuration')
    require(all(int(entry.split(',')[2]) * 4 == expected for entry, expected in zip(
                ioc['FREERTOS.Tasks01'].split(';'), (2048, 2048, 1536, 1024))),
            'task stack word counts agree with documented byte budgets (6656B total)')
    require(not re.search(r'\b(malloc|calloc|realloc|free|pvPortMalloc|vPortFree)\s*\(',
                          application + read('BSP/C30D/c30d_safety.c')),
            'maintained task/BSP paths have no heap allocation or ownership transfers')
    require('HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)' in read('Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c')
            and '#define vPortSVCHandler    SVC_Handler' in cfg
            and '#define xPortPendSVHandler PendSV_Handler' in cfg
            and 'void SVC_Handler' not in irq and 'void PendSV_Handler' not in irq,
            'HAL priority grouping and single SVC/PendSV kernel vector ownership agree')
    for ok, description in checks:
        print(('PASS' if ok else 'FAIL') + ': ' + description)
    print(f'\n{sum(ok for ok, _ in checks)}/{len(checks)} static checks passed; no compiler/probe invoked.')
    return 0 if all(ok for ok, _ in checks) else 1


if __name__ == '__main__':
    raise SystemExit(main())
