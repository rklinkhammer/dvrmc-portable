#pragma once
#include <cstdint>
uint32_t dvrmc_top(uint32_t address,uint32_t value,bool write_enable,bool control_tick,
                   bool reset,uint32_t pos0,uint32_t pos1,uint32_t errors0,uint32_t errors1,
                   int32_t& duty0,int32_t& duty1);
