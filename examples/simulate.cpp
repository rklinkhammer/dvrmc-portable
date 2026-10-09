#include "dvrmc/host.hpp"
#include <cmath>
#include <iostream>
using namespace dvrmc;
int main() {
    ModelIo io; MotorController controller(io);
    auto motor=MotorProperties::jga25_370_6v_130rpm(0); // wheel radius unknown
    PidParameters pid; pid.kp=.005; pid.ki=.02; pid.feedforward=1.0/130;
    // EXAMPLE GAINS FOR THE TOY MODEL ONLY; not tuned for physical motors.
    auto left=make_config(motor,pid,{});
    Calibration trim; trim.forward_gain=1.1; trim.reverse_gain=1.08;
    auto right=make_config(motor,pid,trim);
    controller.configure(0,left); controller.configure(1,right);
    controller.speed(0,60000); controller.speed(1,60000); controller.commit();
    double rpm[2]={},counts[2]={}; EncoderSnapshot feedback[2];
    for(unsigned tick=0;tick<5000;++tick) {
        if(tick%20==0) controller.keepalive();
        for(unsigned m=0;m<2;++m) {
            double loaded_max=m==0 ? 130.0:115.0;
            rpm[m]+=(loaded_max*io.device.duty(m)/one-rpm[m])*.001/.08;
            counts[m]+=rpm[m]*2059.2/60*.001;
            feedback[m].position=uint32_t(int64_t(std::llround(counts[m])));
        }
        io.tick(feedback);
    }
    auto s=controller.snapshot();
    for(unsigned m=0;m<2;++m)
        std::cout<<"motor "<<m<<": "<<s.motors[m].speed_mrpm/1000.0
                 <<" RPM, duty "<<s.motors[m].duty_q16/double(one)<<'\n';
    controller.stop();
    for(unsigned m=0;m<2;++m)
        if(std::abs(s.motors[m].speed_mrpm-60000)>4000 || s.motors[m].faults) return 1;
    if(io.device.duty(0) || io.device.duty(1)) return 2;
}
