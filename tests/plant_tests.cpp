#include "dvrmc/host.hpp"
#include "dvrmc/sim/plant.hpp"
#include <iostream>
#include <stdexcept>
using namespace dvrmc;
void check(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
struct Bench {
    ModelIo io; MotorController controller{io}; sim::Rig rig;
    Bench() {
        PidParameters pid;pid.kp=.004;pid.ki=.015;pid.feedforward=1.0/130;
        auto c=make_config(MotorProperties::jga25_370_6v_130rpm(0),pid,{});
        for(unsigned m=0;m<2;++m) {controller.configure(m,c);controller.speed(m,60000);}
        controller.commit();
    }
    void run(unsigned ms,bool keepalive=true) {
        for(unsigned t=0;t<ms;++t) {
            if(keepalive && t%20==0) controller.keepalive();
            auto e=rig.feedback();io.tick(e.data());
            rig.advance(1000,io.device.duty(0),io.device.duty(1));
        }
    }
};
int main() {try {
    sim::Quadrature decoder(3);
    decoder.sample(3);decoder.sample(0); // short two-bit glitch rejected
    check(decoder.feedback().errors==0,"glitch filter");
    for(unsigned ab:{1u,3u,2u,0u}) for(int i=0;i<3;++i) decoder.sample(ab);
    check(decoder.feedback().position==4,"forward quadrature");
    for(unsigned ab:{2u,3u,1u,0u,2u}) for(int i=0;i<3;++i) decoder.sample(ab);
    check(decoder.feedback().position==UINT32_MAX,"reverse and counter wrap");
    for(int i=0;i<3;++i) decoder.sample(1);
    check(decoder.feedback().errors==1,"illegal transition diagnostic");
    sim::Motor pwm;unsigned high=0;
    for(unsigned i=0;i<1000;++i) {pwm.step(one/2);high+=pwm.pwm;}
    check(high==500,"PWM half duty");
    sim::Motor motor;
    for(unsigned i=0;i<1000000;++i) motor.step(one);
    double expected=motor.rotor_angle*44/(2*sim::pi);
    check(std::abs(double(motor.decoder.feedback().position)-expected)<2,"Hall position follows rotor angle");
    check(motor.rpm()>110 && motor.rpm()<140,"full voltage speed sanity");
    motor.faults.locked_shaft=true;
    for(unsigned i=0;i<10000;++i) motor.step(one);
    check(motor.rpm()==0 && std::abs(motor.current_a-1)<.001,"locked rotor stall current V/R");
    motor.faults.driver_open=true;motor.step(one);
    check(motor.current_a==0 && motor.terminal_v==0,"open bridge");
    sim::Motor glitch;
    glitch.faults.hall_xor=3;
    for(unsigned i=0;i<3;++i) glitch.step(0);
    check(glitch.decoder.feedback().errors==1,"injected two-bit Hall error");
    sim::Motor sag;
    sag.faults.supply_scale=.5;
    for(unsigned i=0;i<1000000;++i) sag.step(one);
    check(sag.rpm()>50 && sag.rpm()<75,"supply sag reduces available speed");
    sim::Motor swapped;swapped.faults.swap_ab=true;
    for(unsigned i=0;i<100000;++i) swapped.step(one);
    check(signed_word(swapped.decoder.feedback().position)<0 && swapped.rpm()>0,"swapped Hall polarity");
    Bench b;b.run(2200);
    for(const auto& m:b.rig.motors) check(std::abs(m.rpm()-60)<5,"closed loop tracking");
    b.rig.motors[1].faults.load_nm=.035;b.run(1200);
    check(std::abs(b.rig.motors[1].rpm()-60)<6,"load rejection");
    for(unsigned m=0;m<2;++m) b.controller.speed(m,-45000);
    b.controller.commit();b.run(2200);
    for(const auto& m:b.rig.motors) check(std::abs(m.rpm()+45)<6,"reverse tracking");
    b.rig.motors[0].faults.locked_shaft=true;b.run(400);
    check(b.rig.motors[0].rpm()==0,"stall injection");
    check(b.controller.snapshot().motors[0].faults==0,"stall detection is not implemented");
    b.rig.motors[0].faults.locked_shaft=false;b.run(1600);
    check(std::abs(b.rig.motors[0].rpm()+45)<7,"post-stall recovery");
    b.rig.motors[1].faults.freeze_hall=true;b.run(10);
    auto count=b.rig.feedback()[1].position;b.run(200);
    check(b.rig.feedback()[1].position==count,"frozen Hall feedback");
    check(std::abs(b.rig.motors[1].rpm())>10,"physical rotation continues with frozen Hall");
    b.rig.motors[1].faults.freeze_hall=false;b.run(500,false);
    for(const auto& s:b.controller.snapshot().motors)
        check((s.faults&WATCHDOG)&&s.duty_q16==0,"watchdog disables output");
    Bench stopped;stopped.run(100);stopped.controller.stop();stopped.run(1);
    check(stopped.io.device.duty(0)==0 && !stopped.rig.motors[0].pwm,"stop reaches PWM");
    std::cout<<"PASS: PWM, mechanics, Hall decoding, tracking, load, reverse, stall, sensor loss, watchdog, stop\n";
    return 0;
} catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
