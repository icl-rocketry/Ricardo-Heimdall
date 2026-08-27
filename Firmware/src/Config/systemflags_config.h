#pragma once

#include <stdint.h>
#include <type_traits>

enum class SYSTEM_FLAG:uint32_t{
    //state flags
    STATE_IDLE = (1 << 0), 
    //critical messages 
    ERROR_SERIAL = (1 << 10),
    ERROR_CAN = (1<<11),
    ERROR_SD = (1<<19),
    ERROR_PGOOD = (1<<20),
    ERROR_VBOUNDS = (1<<21)
};

using system_flag_t = uint32_t;

