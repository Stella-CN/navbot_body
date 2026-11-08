#pragma once
#include <cstdint>
struct MockDebug
{
    uint32_t APB1FZ{};
};
struct MockWatchdog
{
    uint32_t KR{}, PR{}, RLR{}, SR{};
};
struct MockDwt
{
    uint32_t CYCCNT{};
};
extern MockDebug mock_debug;
extern MockWatchdog mock_watchdog;
extern MockDwt mock_dwt;
#define DBGMCU (&mock_debug)
#define IWDG (&mock_watchdog)
#define DWT (&mock_dwt)
#define DBGMCU_APB1_FZ_DBG_IWDG_STOP (1U << 12)
inline uint32_t HAL_GetTick() { return 0; }
