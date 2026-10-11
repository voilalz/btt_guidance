#include "boost_cruise.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
static void check(int ok,const char *name){if(!ok){fprintf(stderr,"FAIL: %s\n",name);++failures;}}
static void analytic(BCParams *p,BCState *s){
    bc_default(p);memset(s,0,sizeof(*s));s->a[6]=1;s->a[2]=-100;
    s->a[15]=p->boost_fuel0;s->a[16]=p->cruise_fuel0;
    s->a[13]=p->vehicle.mass_dry+s->a[15]+s->a[16];
    s->boost_active=1;s->airborne=1;s->ignition_time=-1;
    p->vehicle.g=0;p->vehicle.aero_enabled=0;p->stabilize=0;
}
static void gate_tests(void){
    BCParams p;BCState s;BCObservation o;double used,expected,k,m0;
    int rc;analytic(&p,&s);p.ignition_height=50;p.ignition_speed=20;
    k=p.boost_fuel0/p.burn_duration;m0=s.a[13];
    expected=m0/k*(1-exp(-k*20/p.boost_thrust));
    check(bc_observe(0,&s,&p,&o)==0&&o.height_gate&&!o.speed_gate,"height alone is not ready");
    used=0;rc=bc_advance(0,2,&s,&p,&used);
    check(rc==BC_IGNITION&&fabs(used-expected)<1e-9,"analytic first AND-gate ignition time");
    check(s.cruise_ignited&&fabs(s.ignition_time-expected)<1e-9,"ignition latch and time");
    check(fabs(s.a[3]-20)<1e-8&&s.a[14]==0,"handoff preserves velocity and zero initial cruise thrust");
    check(s.boost_active&&s.a[15]>0,"solid booster continues after cruise ignition");
    p.stabilize=1;p.cruise_speed=1;bc_observe(used,&s,&p,&o);
    check(o.flight.thrust_cmd>0,"ignited cruise engine establishes idle thrust under overspeed");
    s.a[2]=-1;s.a[3]=1;bc_observe(used,&s,&p,&o);
    check(s.cruise_ignited&&o.stage==BC_OVERLAP,"ignition remains latched below gates");
    analytic(&p,&s);p.ignition_height=1000;p.ignition_speed=1;
    check(bc_advance(0,1,&s,&p,&used)==0&&!s.cruise_ignited,"speed alone does not ignite");
    analytic(&p,&s);s.a[3]=30;s.a[5]=1;p.ignition_height=100;p.ignition_speed=20;
    rc=bc_advance(0,.005,&s,&p,&used);
    check(rc==BC_IGNITION&&used==0&&s.cruise_ignited&&s.ignition_time==0&&s.a[2]==-100&&s.a[14]==0,
          "ready descending start ignites immediately even when end height would be below gate");
    if(!failures)puts("PASS: AND gates, analytic ignition, latch and continuous handoff");
}
static void burnout_tests(void){
    BCParams p;BCState s;BCObservation o;double used,k,m0,v,position;
    int rc;analytic(&p,&s);p.ignition_height=1000;
    k=p.boost_fuel0/p.burn_duration;m0=s.a[13];
    rc=bc_advance(0,6,&s,&p,&used);
    v=p.boost_thrust/k*log(m0/(m0-p.boost_fuel0));
    position=p.boost_thrust/k*(p.burn_duration-(m0-p.boost_fuel0)/k*log(m0/(m0-p.boost_fuel0)));
    check(rc==BC_BURNOUT&&used==p.burn_duration,"known solid burnout splits a long step");
    check(!s.boost_active&&s.a[15]==0,"booster inventory empty and thrust off");
    /* A single long RK4 has truncation error; repeat with small steps below. */
    check(bc_observe(used,&s,&p,&o)==0&&o.boost_thrust==0&&o.stage==BC_COAST,"unmet gates lead to coast");
    analytic(&p,&s);p.ignition_height=1000;
    {double t=0;int i;for(i=0;i<400;++i){rc=bc_advance(t,.01,&s,&p,&used);check(rc>=0,"small-step boost valid");t+=used;}}
    check(fabs(s.a[3]-v)<1e-8,"analytic variable-mass booster velocity");
    check(fabs(s.a[0]-position)<1e-8,"analytic variable-mass booster position");
    check(fabs(s.a[13]-(p.vehicle.mass_dry+s.a[15]+s.a[16]))<1e-13,"independent inventories sum to total mass");
    if(!failures)puts("PASS: solid burnout, analytic variable mass and no false cruise");
}
static void fuel_ground_tests(void){
    BCParams p;BCState s;BCObservation o;double used,expected;
    int rc;analytic(&p,&s);s.boost_active=0;s.a[15]=0;s.cruise_ignited=1;s.ignition_time=0;
    s.a[16]=.01;s.a[13]=p.vehicle.mass_dry+.01;p.vehicle.fuel_c0=.02;p.vehicle.fuel_cT=0;
    s.a[14]=10;p.vehicle.throttle_trim=10/p.vehicle.thrust_max;
    rc=bc_advance(0,1,&s,&p,&used);
    check(rc==BC_FUEL_OUT&&fabs(used-.5)<1e-10,"cruise fuel event time");
    check(s.cruise_exhausted&&s.a[16]==0&&s.a[14]==0,"cruise shutdown inventory and thrust");
    check(bc_observe(used,&s,&p,&o)==0&&o.stage==BC_EMPTY,"empty stage recorded");
    analytic(&p,&s);s.a[2]=-10;p.vehicle.g=9.80665;p.ignition_height=1000;
    expected=sqrt(20/p.vehicle.g);rc=bc_advance(0,2,&s,&p,&used);
    check(rc==BC_GROUND&&fabs(used-expected)<1e-9&&s.a[2]==0,"analytic ground event wins before burnout");
    if(!failures)puts("PASS: cruise fuel depletion and ground event");
}
static void launch_invalid_tests(void){
    BCParams p;BCState s;double used,ds[BC_NSTATE];
    bc_default(&p);check(bc_initial(&p,&s)==0,"default static launch initialized");
    check(s.a[2]==0&&s.a[3]==0&&s.a[4]==0&&s.a[5]==0,"actual zero-height zero-speed start");
    check(bc_advance(0,.005,&s,&p,&used)>=0&&s.a[2]<0&&s.airborne,"ground start departs instead of immediate termination");
    bc_default(&p);p.boost_thrust=1;check(bc_initial(&p,&s)<0,"underpowered release fails clearly");
    bc_default(&p);p.ignition_height=NAN;check(bc_initial(&p,&s)<0,"nonfinite gate rejected");
    bc_default(&p);p.burn_duration=0;check(bc_initial(&p,&s)<0,"zero burn duration rejected");
    bc_default(&p);p.cruise_idle_thrust=p.vehicle.thrust_max+1;check(bc_initial(&p,&s)<0,"idle above maximum rejected");
    bc_default(&p);p.cruise_idle_thrust=-1;check(bc_initial(&p,&s)<0,"negative idle rejected");
    bc_default(&p);bc_initial(&p,&s);s.a[15]=-.1;check(bc_advance(0,.01,&s,&p,&used)<0,"negative fuel rejected");
    bc_default(&p);bc_initial(&p,&s);p.pitch_gain=NAN;check(bc_rhs(0,&s,&p,ds)<0,"nonfinite controller rejected");
    if(!failures)puts("PASS: static launch, insufficient lift-off and invalid parameters");
}
static void multiple_event_tests(void){
    BCParams p;BCState s;double t=3.9,used;int rc;
    analytic(&p,&s);s.a[15]=(p.burn_duration-t)*p.boost_fuel0/p.burn_duration;
    s.a[13]=p.vehicle.mass_dry+s.a[15]+s.a[16];s.a[3]=30;
    p.ignition_height=50;p.ignition_speed=20;
    rc=bc_advance(t,.2,&s,&p,&used);
    check(rc==BC_IGNITION&&used==0,"already satisfied gates ignite without wasting remaining window");
    rc=bc_advance(t,.2,&s,&p,&used);t+=used;
    check(rc==BC_BURNOUT&&fabs(t-4)<1e-12,"burnout follows ignition in same requested window");
    rc=bc_advance(t,4.1-t,&s,&p,&used);t+=used;
    check(rc==0&&fabs(t-4.1)<1e-12&&s.cruise_ignited&&!s.boost_active&&s.a[14]>0,
          "remaining window advances in cruise with continuous thrust");
    if(!failures)puts("PASS: multiple events and remaining time in one integration window");
}
int main(void){gate_tests();burnout_tests();fuel_ground_tests();launch_invalid_tests();multiple_event_tests();return failures?1:0;}
