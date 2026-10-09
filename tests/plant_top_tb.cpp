#include "dvrmc/top.hpp"
#include "dvrmc/registers.hpp"
#include "dvrmc/sim/plant.hpp"
#include <iostream>
// No host API dependency: identical transaction interface in C and RTL cosim.
int main() {
    using namespace dvrmc;
    sim::Rig rig;int32_t duty0=0,duty1=0;
    auto call=[&](uint32_t address,uint32_t value,bool write,bool tick,bool clear=false) {
        auto e=rig.feedback();
        return dvrmc_top(address,value,write,tick,clear,e[0].position,e[1].position,
                        e[0].errors,e[1].errors,duty0,duty1);
    };
    call(reg::ID,0,false,false,true);
    MotorConfig c;
    c[field::ENCODER_PPR]=11;c[field::GEAR_NUM]=468;c[field::GEAR_DEN]=10;
    c[field::COUNTS_PER_REV_Q16]=134951731;c[field::MAX_SPEED_MRPM]=130000;
    c[field::MAX_ACCEL_MRPM_S]=130000;
    c[field::KP_Q16]=262;c[field::KI_Q16]=983;c[field::KFF_Q16]=504;
    c[field::ENABLE]=1;c[field::TARGET_MRPM]=60000;
    for(unsigned m=0;m<2;++m) for(unsigned f=0;f<field::COUNT;++f)
        call(reg::config(m,field::Index(f)),uint32_t(c.words[f]),true,false);
    if(!valid(c)) {std::cerr<<"Invalid test configuration\n";return 3;}
    call(reg::COMMIT,1,true,false);
    for(unsigned ms=0;ms<2500;++ms) {
        if(ms%20==0) call(reg::KEEPALIVE,1,true,false);
        call(reg::ID,0,false,true);
        rig.advance(1000,duty0,duty1);
    }
    for(const auto& motor:rig.motors) if(std::abs(motor.rpm()-60)>6) {std::cerr<<"Tracking failed: "<<motor.rpm()<<" RPM\n";return 1;}
    for(unsigned ms=0;ms<150;++ms) {
        call(reg::ID,0,false,true);
        rig.advance(1000,duty0,duty1);
    }
    call(reg::SNAPSHOT,1,true,false);
    for(unsigned m=0;m<2;++m)
        if(!(call(reg::telemetry(m,reg::FAULTS),0,false,false)&WATCHDOG)) return 4;
    if(duty0 || duty1) return 5;
    call(reg::STOP,1,true,false);
    if(duty0 || duty1) return 2;
    std::cout<<"PASS: HLS top drives virtual PWM/motor/Hall closed loop\n";
}
