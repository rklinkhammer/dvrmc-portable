#include "dvrmc/top.hpp"
#include "dvrmc/device.hpp"
uint32_t dvrmc_top(uint32_t address,uint32_t value,bool write_enable,bool control_tick,
                   bool reset,uint32_t pos0,uint32_t pos1,uint32_t errors0,uint32_t errors1,
                   int32_t& duty0,int32_t& duty1) {
#ifdef DVRMC_VITIS
#pragma HLS INTERFACE ap_ctrl_hs port=return
#endif
#ifdef DVRMC_SMARTHLS
#pragma HLS function top
#endif
    static dvrmc::Device device;
    if(reset) device.reset();
    else {
        if(write_enable) device.write(address,value);
        if(control_tick) {
            dvrmc::EncoderSnapshot e[dvrmc::motor_count];
            e[0].position=pos0; e[1].position=pos1;
            e[0].errors=errors0; e[1].errors=errors1;
            device.tick(e);
        }
    }
    duty0=device.duty(0); duty1=device.duty(1);
    return device.read(address);
}
