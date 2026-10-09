#include "dvrmc/host.hpp"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
using namespace dvrmc;
static unsigned checks=0;
static void check(bool ok,const char* what) {
    ++checks; if (!ok) { std::cerr<<"FAIL: "<<what<<'\n'; std::exit(1); }
}
static MotorConfig config() {
    auto c=make_config(MotorProperties::jga25_370_6v_130rpm(0),PidParameters{},Calibration{});
    c[field::COUNTS_PER_REV_Q16]=1000*one; // exact test scale
    c[field::MAX_ACCEL_MRPM_S]=10000000;
    c[field::KP_Q16]=one/100;
    c[field::SPEED_ALPHA_Q16]=one;
    return c;
}
int main() {
    auto preset=make_config(MotorProperties::jga25_370_6v_130rpm(30000),{},{});
    check(preset[field::COUNTS_PER_REV_Q16]==134951731,"motor preset exact ratio and x4 scaling");
    check(preset[field::ENABLE]==0 && preset[field::KP_Q16]==0,"preset disabled and untuned");
    try { auto bad=config(); bad[field::KP_Q16]=-1; check(!valid(bad),"negative gain rejected"); }
    catch (...) { check(false,"unexpected exception"); }
    ModelIo io; MotorController host(io);
    check(io.read32(reg::STATUS)==reg::STOPPED,"boot stopped");
    auto c=config(); c[field::ENABLE]=1; c[field::TARGET_MRPM]=10000;
    host.configure(0,c); auto right=c; right[field::TARGET_MRPM]=-10000; host.configure(1,right);
    uint32_t token=host.commit();
    check(!host.applied(token) && io.device.duty(0)==0,"commit deferred");
    host.speed(0,20000); // shadow edit cannot alter captured pending transaction
    EncoderSnapshot feedback[2]; io.tick(feedback);
    check(host.applied(token),"commit acknowledged at tick");
    auto s=host.snapshot();
    check(s.motors[0].target_mrpm==10000 && s.motors[1].target_mrpm==-10000,"atomic independent targets");
    check(s.motors[0].duty_q16>0 && s.motors[1].duty_q16<0,"direction outputs");
    feedback[0].position=1; feedback[1].position=uint32_t(-2); io.tick(feedback);
    auto old=io.read32(reg::telemetry(0,reg::POS_LO));
    check(old==0,"snapshot frozen until requested");
    s=host.snapshot();
    check(s.motors[0].position==1 && s.motors[1].position==-2,"signed independent positions");
    check(s.motors[0].speed_mrpm==60000 && s.motors[1].speed_mrpm==-120000,"speed units");
    host.stop(); check(io.device.duty(0)==0 && io.device.duty(1)==0,"stop without tick");
    // Pending command must not undo stop.
    host.commit(); host.stop(); io.tick(feedback);
    check(io.device.duty(0)==0 && (io.read32(reg::STATUS)&reg::STOPPED),"stop cancels pending enable");
    // Invalid transaction leaves previous active state intact.
    io.write32(reg::config(0,field::DUTY_LIMIT_Q16),uint32_t(-1));
    io.write32(reg::COMMIT,99);
    check(io.read32(reg::STATUS)&reg::REJECTED,"invalid register transaction rejected");
    // Counter wrap, glitch detection, calibration and antiwindup.
    WheelState ws; auto w=config(); w[field::ENABLE]=1; w[field::TARGET_MRPM]=10000;
    EncoderSnapshot e; e.position=0xfffffffeu; wheel_step(w,e,1000,false,ws);
    e.position=1; wheel_step(w,e,1000,false,ws);
    check(ws.position==3,"32-bit encoder wrap");
    e.position+=5000; wheel_step(w,e,1000,false,ws);
    check((ws.faults&BAD_FEEDBACK) && ws.duty_q16==0,"implausible jump latches fault");
    w[field::SPEED_ALPHA_Q16]=one; w[field::PWM_GAIN_FWD_Q16]=one*2;
    w[field::PWM_OFFSET_FWD_Q16]=one/10; ws=WheelState{}; e={};
    wheel_step(w,e,1000,false,ws);
    check(ws.duty_q16==(int64_t(w[field::KP_Q16])*10*2+one/10),"positive PWM calibration");
    w[field::TARGET_MRPM]=-10000;
    w[field::PWM_GAIN_REV_Q16]=one/2; w[field::PWM_OFFSET_REV_Q16]=one/20;
    ws=WheelState{}; wheel_step(w,e,1000,false,ws);
    check(ws.duty_q16==-(int64_t(w[field::KP_Q16])*10/2+one/20),"reverse PWM calibration");
    w[field::TARGET_MRPM]=10000;
    w[field::DRIVE_SIGN]=-1; ws=WheelState{}; wheel_step(w,e,1000,false,ws);
    check(ws.duty_q16<0,"drive polarity after calibration");
    w=config(); w[field::ENABLE]=1; w[field::TARGET_MRPM]=130000;
    w[field::KP_Q16]=one; w[field::KI_Q16]=one; ws=WheelState{};
    for(int i=0;i<200;++i) wheel_step(w,e,1000,false,ws);
    check(ws.duty_q16==one && ws.integral==0,"saturation prevents integral windup");
    w[field::TARGET_MRPM]=0; wheel_step(w,e,1000,true,ws);
    check(ws.duty_q16==0 && (ws.faults&WATCHDOG),"watchdog disables output");
    // Host watchdog and explicit disabled commit fault acknowledgement.
    io.device.reset(); host.configure(0,c); host.configure(1,c); host.commit();
    feedback[0]={}; feedback[1]={};
    for(int i=0;i<102;++i) io.tick(feedback);
    check(host.snapshot().motors[0].faults&WATCHDOG,"register engine watchdog");
    host.keepalive(); io.tick(feedback);
    check(io.device.duty(0)==0,"keepalive does not clear latched fault");
    host.speed(0,0,false); host.speed(1,0,false); host.commit(); io.tick(feedback);
    check(host.snapshot().motors[0].faults==0,"disabled commit acknowledges fault");
    // Coordinated drive geometry and common scaling.
    DifferentialDrive drive(host,.2,.03,.03,130,130);
    token=drive.velocity(.1,0); io.tick(feedback);
    check(host.applied(token),"differential command commit");
    check(io.read32(reg::config(0,field::TARGET_MRPM))==io.read32(reg::config(1,field::TARGET_MRPM)),"straight targets equal");
    token=drive.velocity(0,1); io.tick(feedback);
    check(signed_word(io.read32(reg::config(0,field::TARGET_MRPM)))<0 &&
          signed_word(io.read32(reg::config(1,field::TARGET_MRPM)))>0,"in-place rotation targets");
    // New speed commands must not reset an established integrator.
    ModelIo integral_io; MotorController integral_host(integral_io);
    auto ic=config(); ic[field::KP_Q16]=0; ic[field::KI_Q16]=one;
    ic[field::ENABLE]=1; ic[field::TARGET_MRPM]=10000;
    integral_host.configure(0,ic); integral_host.configure(1,ic); integral_host.commit();
    for(int i=0;i<10;++i) integral_io.tick(feedback);
    auto before=integral_host.snapshot().motors[0].duty_q16;
    integral_host.speed(0,10000); integral_host.commit(); integral_io.tick(feedback);
    check(integral_host.snapshot().motors[0].duty_q16>before,"target-only commit preserves integral");
    // Calibration changes measured speed without altering raw position units.
    w=config(); w[field::COUNTS_PER_REV_Q16]=2000*one; ws=WheelState{}; e={};
    wheel_step(w,e,1000,false,ws); e.position=1; wheel_step(w,e,1000,false,ws);
    check(ws.position==1 && ws.speed_mrpm==30000,"resolution calibration affects speed only");
    // Exercise representable extremes under UBSan, including negative inputs.
    w=config(); w[field::ENABLE]=1; w[field::MAX_SPEED_MRPM]=1000000;
    w[field::TARGET_MRPM]=-1000000; w[field::MAX_DELTA_COUNTS]=65535;
    w[field::COUNTS_PER_REV_Q16]=one;
    w[field::KP_Q16]=w[field::KI_Q16]=w[field::KD_Q16]=16*one;
    ws=WheelState{}; e={}; wheel_step(w,e,100,false,ws);
    for(int i=0;i<100;++i) { e.position+=i%2 ? 65535u : uint32_t(-65535); wheel_step(w,e,100,false,ws); }
    check(ws.duty_q16<=one && ws.duty_q16>=-one,"extreme arithmetic stays bounded");
    std::cout<<"PASS: "<<checks<<" controller/driver checks\n";
}
