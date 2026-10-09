#include "dvrmc/top.hpp"
#include "dvrmc/registers.hpp"
#include <iostream>
using namespace dvrmc;
int main() {
    int32_t a=0,b=0;
    auto call=[&](uint32_t address,uint32_t value,bool write,bool tick,bool reset=false) {
        return dvrmc_top(address,value,write,tick,reset,0,0,0,0,a,b);
    };
    if(call(reg::ID,0,false,false,true)!=reg::MAGIC) return 1;
    MotorConfig c;
    c[field::ENCODER_PPR]=11; c[field::GEAR_NUM]=468; c[field::GEAR_DEN]=10;
    c[field::COUNTS_PER_REV_Q16]=134951731;
    c[field::MAX_SPEED_MRPM]=130000; c[field::MAX_ACCEL_MRPM_S]=130000;
    c[field::KP_Q16]=1000; c[field::ENABLE]=1; c[field::TARGET_MRPM]=10000;
    for(unsigned m=0;m<motor_count;++m)
        for(unsigned f=0;f<field::COUNT;++f)
            call(reg::config(m,field::Index(f)),uint32_t(c.words[f]),true,false);
    call(reg::COMMIT,7,true,false);
    if(a || b) return 2;
    for(unsigned i=0;i<20;++i) call(reg::ID,0,false,true);
    if(a<=0 || a!=b || call(reg::APPLIED,0,false,false)!=7) return 3;
    call(reg::STOP,1,true,false);
    if(a || b) return 4;
    std::cout<<"PASS: shared HLS top register/control transactions\n";
    return 0;
}
