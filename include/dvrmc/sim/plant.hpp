#pragma once
#include "dvrmc/types.hpp"
#include <array>
#include <cmath>
#include <stdexcept>

// Host/testbench-only plant. Never include this file in synthesizable sources.
namespace dvrmc { namespace sim {
constexpr double pi=3.14159265358979323846;
struct Parameters {
    // Illustrative estimates, NOT measured JGA25-370 electrical data.
    double supply_v=6, resistance_ohm=6, inductance_h=.003;
    double torque_nm_per_a=.0094, back_emf_v_per_rad_s=.0094;
    double rotor_inertia_kg_m2=.000002, viscous_nm_per_rad_s=.0000002;
    double coulomb_nm=.0001, gear_ratio=46.8, gear_efficiency=.75;
    double load_inertia_kg_m2=.0001;
    unsigned encoder_ppr=11, pwm_hz=20000, filter_samples=3;
    void validate() const {
        const double positive[]={supply_v,resistance_ohm,inductance_h,torque_nm_per_a,
            back_emf_v_per_rad_s,rotor_inertia_kg_m2,gear_ratio,gear_efficiency};
        for(double x:positive) if(!std::isfinite(x)||x<=0) throw std::invalid_argument("invalid plant parameter");
        const double nonnegative[]={viscous_nm_per_rad_s,coulomb_nm,load_inertia_kg_m2};
        for(double x:nonnegative) if(!std::isfinite(x)||x<0) throw std::invalid_argument("invalid plant parameter");
        if(gear_efficiency>1 || !encoder_ppr || !pwm_hz || pwm_hz>1000000 ||
           1000000%pwm_hz || !filter_samples || filter_samples>32)
            throw std::invalid_argument("PWM must divide 1 MHz; filter is 1..32 samples");
    }
};
struct Faults {
    bool locked_shaft=false, driver_open=false, freeze_hall=false, swap_ab=false;
    bool stuck_a=false, stuck_b=false;
    unsigned hall_xor=0; // deterministic glitches: XOR bit 1=A, bit 0=B
    double supply_scale=1, load_nm=0; // load magnitude at output shaft, opposes motion
};
class Quadrature {
public:
    explicit Quadrature(unsigned samples=3): required_(samples) {
        if(!samples || samples>32) throw std::invalid_argument("invalid filter length");
    }
    void sample(unsigned ab) {
        ab &= 3;
        if(ab!=candidate_) {candidate_=ab; run_=1;} else if(run_<required_) ++run_;
        if(run_<required_ || ab==stable_) return;
        static const int delta[16]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
        if((ab^stable_)==3) ++feedback_.errors;
        else feedback_.position+=uint32_t(delta[stable_*4+ab]);
        stable_=ab;
    }
    EncoderSnapshot feedback() const {return feedback_;}
private:
    unsigned required_,candidate_=0,stable_=0,run_=0;
    EncoderSnapshot feedback_;
};
class Motor {
public:
    explicit Motor(Parameters parameters=Parameters{}): p(parameters),decoder(parameters.filter_samples) {p.validate();decay_=std::exp(-p.resistance_ohm*.000001/p.inductance_h);}
    const Parameters p;
    Faults faults;
    Quadrature decoder;
    double current_a=0, rotor_rad_s=0, rotor_angle=0;
    double terminal_v=0;
    unsigned hall=0;
    bool pwm=false;
    int direction=0;
    double rpm() const {return rotor_rad_s*60/(2*pi*p.gear_ratio);}
    double output_turns() const {return rotor_angle/(2*pi*p.gear_ratio);}
    // One microsecond: explicit switching, electrical dynamics, mechanics, Hall sampling.
    void step(int32_t duty_q16) {
        if(!std::isfinite(faults.supply_scale)||faults.supply_scale<0 ||
           !std::isfinite(faults.load_nm)||faults.load_nm<0 || faults.hall_xor>3)
            throw std::invalid_argument("invalid fault injection");
        const unsigned period=1000000/p.pwm_hz;
        if(phase_==0) latched_=duty_q16>one?one:(duty_q16<-one?-one:duty_q16);
        direction=(latched_>0)-(latched_<0);
        pwm=uint64_t(phase_)*one < uint64_t(latched_<0?-int64_t(latched_):latched_)*period;
        terminal_v=(!faults.driver_open && pwm)?direction*p.supply_v*faults.supply_scale:0;
        constexpr double dt=.000001;
        if(faults.driver_open) current_a=0; // ideal high impedance; inductive transient omitted
        else {
            double equilibrium=(terminal_v-p.back_emf_v_per_rad_s*rotor_rad_s)/p.resistance_ohm;
            current_a=equilibrium+(current_a-equilibrium)*decay_;
        }
        if(faults.locked_shaft) rotor_rad_s=0;
        else {
            double drive=p.torque_nm_per_a*current_a-p.viscous_nm_per_rad_s*rotor_rad_s;
            double friction=p.coulomb_nm+faults.load_nm/(p.gear_ratio*p.gear_efficiency);
            double sign=rotor_rad_s==0 ? ((drive>0)-(drive<0)) : ((rotor_rad_s>0)-(rotor_rad_s<0));
            double inertia=p.rotor_inertia_kg_m2+p.load_inertia_kg_m2/(p.gear_ratio*p.gear_ratio);
            double next=rotor_rad_s+(drive-sign*friction)*dt/inertia;
            if((rotor_rad_s==0 || next*rotor_rad_s<0) && std::abs(drive)<=friction) next=0;
            rotor_rad_s=next;
        }
        rotor_angle+=rotor_rad_s*dt;
        // Bound the phase before integer conversion, including reverse rotation.
        double edge=std::floor(rotor_angle*(4.0*p.encoder_ppr)/(2*pi));
        int index=int(std::fmod(edge,4)); if(index<0) index+=4;
        static const unsigned gray[4]={0,1,3,2};
        if(!faults.freeze_hall) held_hall_=gray[index];
        hall=held_hall_;
        if(faults.swap_ab) hall=((hall&1)<<1)|((hall&2)>>1);
        if(faults.stuck_a) hall &= 1;
        if(faults.stuck_b) hall &= 2;
        hall ^= faults.hall_xor;
        decoder.sample(hall);
        phase_=(phase_+1)%period;
    }
private:
    double decay_=0;
    unsigned phase_=0,held_hall_=0;
    int32_t latched_=0;
};
class Rig {
public:
    explicit Rig(Parameters left=Parameters{},Parameters right=Parameters{}): motors{{Motor(left),Motor(right)}} {}
    std::array<Motor,2> motors;
    uint64_t time_us=0;
    template<class Observer>
    void advance(unsigned microseconds,int32_t left,int32_t right,Observer observe) {
        for(unsigned t=0;t<microseconds;++t) {
            motors[0].step(left);motors[1].step(right);++time_us;observe(*this);
        }
    }
    void advance(unsigned microseconds,int32_t left,int32_t right) {
        advance(microseconds,left,right,[](const Rig&){});
    }
    std::array<EncoderSnapshot,2> feedback() const {return {{motors[0].decoder.feedback(),motors[1].decoder.feedback()}};}
};
}}
