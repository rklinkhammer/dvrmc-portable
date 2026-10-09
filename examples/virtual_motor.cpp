#include "dvrmc/host.hpp"
#include "dvrmc/sim/plant.hpp"
#include <fstream>
#include <iostream>
using namespace dvrmc;
int main(int argc,char** argv) {
    if(argc>3) {std::cerr<<"Usage: virtual_motor [trace.csv [signals.vcd]]\n";return 2;}
    std::ofstream csv(argc>=2?argv[1]:"virtual_motor.csv");
    if(!csv) {std::cerr<<"Cannot open trace\n";return 2;}
    std::ofstream vcd;
    if(argc==3) {
        vcd.open(argv[2]);if(!vcd) {std::cerr<<"Cannot open waveform\n";return 2;}
        vcd<<"$timescale 1us $end\n$scope module motors $end\n";
        const char* names[]={"left_pwm","left_reverse","left_hall_a","left_hall_b",
                             "right_pwm","right_reverse","right_hall_a","right_hall_b"};
        for(unsigned i=0;i<8;++i) vcd<<"$var wire 1 "<<char('!'+i)<<' '<<names[i]<<" $end\n";
        vcd<<"$upscope $end\n$enddefinitions $end\n#0\n$dumpvars\n";
        for(unsigned i=0;i<8;++i) vcd<<'0'<<char('!'+i)<<'\n';
        vcd<<"$end\n";
    }
    unsigned last_signals=0;
    ModelIo io; MotorController controller(io); sim::Rig rig;
    PidParameters pid;pid.kp=.004;pid.ki=.015;pid.feedforward=1.0/130;
    auto config=make_config(MotorProperties::jga25_370_6v_130rpm(0),pid,{});
    for(unsigned m=0;m<2;++m) {controller.configure(m,config);controller.speed(m,60000);}
    controller.commit();
    csv<<"time_s,event,motor,target_rpm,true_rpm,measured_rpm,output_turns,current_a,duty,hall_a,hall_b,encoder_count,encoder_errors,faults\n";
    const char* event="forward";
    for(unsigned ms=0;ms<7000;++ms) {
        if(ms==1500) {rig.motors[1].faults.load_nm=.035;event="load";}
        if(ms==2500) {controller.speed(0,-45000);controller.speed(1,-45000);controller.commit();event="reverse";}
        if(ms==4000) {rig.motors[0].faults.locked_shaft=true;event="stall";}
        if(ms==4500) {rig.motors[0].faults.locked_shaft=false;event="release";}
        if(ms==5200) {rig.motors[1].faults.freeze_hall=true;event="hall_freeze";}
        if(ms==5500) {rig.motors[1].faults.freeze_hall=false;event="hall_restore";}
        if(ms==6000) event="lost_commands";
        if(ms<6000 && ms%20==0) controller.keepalive();
        auto feedback=rig.feedback();io.tick(feedback.data());
        auto snapshot=controller.snapshot();
        for(unsigned m=0;m<2;++m) {
            const auto& motor=rig.motors[m];const auto& s=snapshot.motors[m];
            csv<<ms*.001<<','<<event<<','<<m<<','<<s.target_mrpm*.001<<','<<motor.rpm()<<','
               <<s.speed_mrpm*.001<<','<<motor.output_turns()<<','<<motor.current_a<<','
               <<s.duty_q16/double(one)<<','<<((motor.hall>>1)&1)<<','<<(motor.hall&1)<<','
               <<feedback[m].position<<','<<s.encoder_errors<<','<<s.faults<<'\n';
        }
        if(argc==3) rig.advance(1000,io.device.duty(0),io.device.duty(1),[&](const sim::Rig& r) {
            unsigned bits=0;
            for(unsigned m=0;m<2;++m) {
                const auto& x=r.motors[m];
                bits |= (unsigned(x.pwm)|(unsigned(x.direction<0)<<1)|
                         (((x.hall>>1)&1)<<2)|((x.hall&1)<<3))<<(m*4);
            }
            if(bits!=last_signals) {
                vcd<<'#'<<r.time_us<<'\n';
                for(unsigned i=0;i<8;++i) if((bits^last_signals)&(1u<<i))
                    vcd<<((bits>>i)&1)<<char('!'+i)<<'\n';
                last_signals=bits;
            }
        });
        else rig.advance(1000,io.device.duty(0),io.device.duty(1));
    }
    auto result=controller.snapshot();
    for(auto& s:result.motors) if(!(s.faults&WATCHDOG)||s.duty_q16) return 1;
    csv.flush();if(argc==3) vcd.flush();
    if(!csv || (argc==3 && !vcd)) return 2;
    std::cout<<"PASS: virtual PWM/motor/Hall scenario; trace: "<<(argc>=2?argv[1]:"virtual_motor.csv")<<'\n';
}
