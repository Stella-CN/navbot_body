#pragma once
#include <cassert>
extern int critical_depth;
#define taskENTER_CRITICAL() (++critical_depth)
#define taskEXIT_CRITICAL()         \
    do                              \
    {                               \
        assert(critical_depth > 0); \
        --critical_depth;           \
    } while (0)
