#include "civil_route.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
static void check(int ok,const char *name){if(!ok){fprintf(stderr,"FAIL: %s\n",name);++failures;}}
static int custom(const CivilFlightView *v,const CivilSegment *g,const CivilControllerMemory *m,CivilReference *r){
    (void)v;(void)m;r->height_m=g->height_m+5;r->speed_mps=g->speed_mps;return 0;
}
static int invalid(const CivilFlightView *v,const CivilSegment *g,const CivilControllerMemory *m,CivilReference *r){
    (void)v;(void)g;(void)m;r->height_m=NAN;r->speed_mps=25;return 0;
}
static const CivilController user_controller={101,"user",custom};
static const CivilController bad_controller={102,"bad",invalid};
static void controller_checks(void){
    CivilFlightView v={0};CivilSegment g={800,80,20};CivilControllerMemory m={0,40,25};CivilReference r;
    const CivilController *c=civil_controller("slew");v.t_s=2;
    check(c&&c->evaluate(&v,&g,&m,&r)==0&&r.height_m==42&&r.speed_mps==24,"slew reference uses elapsed segment time");
    c=civil_controller("direct");check(c&&c->evaluate(&v,&g,&m,&r)==0&&r.height_m==80&&r.speed_mps==20,"direct waypoint reference");
    c=civil_controller("launch");check(c&&c->evaluate(&v,&g,&m,&r)==0&&r.height_m==40&&r.speed_mps==25,"launch reference");
    check(civil_controller("unknown")==NULL,"unknown controller rejected");
    if(!failures)puts("PASS: replaceable direct/slew/launch reference generators");
}
static void traffic_cpa_checks(void){
    CivilConfig c;CivilTraffic a;CivilFlightView v={0};CivilPrediction p;
    civil_default(&c,CIVIL_HIGH);check(civil_traffic(120,&c,&a)==0&&a.x_m==0&&a.height_m==10000,"high traffic analytic constant altitude");
    civil_default(&c,CIVIL_CROSSING);check(civil_traffic(50,&c,&a)==0&&a.x_m==1500&&a.z_m==0&&a.height_m==40,"crossing traffic analytic location");
    v.height_m=40;v.vx_mps=25;a=(CivilTraffic){100,40,-100,0,0,25};
    check(civil_cpa(&v,&a,20,&p)==0&&fabs(p.tcpa_s-4)<1e-12&&p.horizontal_m<1e-12,"analytic crossing closest approach");
    check(civil_cpa(&v,&a,2,&p)==0&&p.tcpa_s==2&&fabs(p.horizontal_m-sqrt(5000))<1e-12,"closest approach clipped to lookahead");
    a=(CivilTraffic){100,40,100,50,0,0};check(civil_cpa(&v,&a,20,&p)==0&&p.tcpa_s==0&&!p.approaching,"receding traffic has no future closest approach");
    a.vx_mps=25;check(civil_cpa(&v,&a,20,&p)==0&&p.tcpa_s==0&&isfinite(p.horizontal_m),"zero relative velocity finite prediction");
    a.x_m=NAN;check(civil_cpa(&v,&a,20,&p)<0,"nonfinite traffic rejected");
    if(!failures)puts("PASS: analytic traffic and horizontal closest approach boundaries");
}
static void supervisor_checks(void){
    CivilConfig c;CivilFlightView v={0};CivilTraffic a={100,40,-100,0,0,25};CivilPrediction p;
    CivilReference nominal={40,25},applied;CivilSafety s={0};int e;
    civil_default(&c,CIVIL_CROSSING);v.height_m=40;v.vx_mps=25;v.cruise_ignited=1;
    e=civil_supervise(0,&v,&a,&c,&nominal,&s,&applied,&p);
    check(e==CIVIL_SAFETY_ON&&s.active&&s.activations==1&&applied.height_m==160,"crossing risk activates vertical separation");
    a=(CivilTraffic){600,40,0,50,0,0};
    e=civil_supervise(39,&v,&a,&c,&nominal,&s,&applied,&p);
    check(e==0&&s.active&&applied.height_m==160,"minimum holding time prevents premature release");
    a.vx_mps=25;
    e=civil_supervise(40,&v,&a,&c,&nominal,&s,&applied,&p);
    check(e==0&&s.active&&applied.height_m==160,"stationary relative traffic keeps override after hold");
    a.vx_mps=50;
    e=civil_supervise(40,&v,&a,&c,&nominal,&s,&applied,&p);
    check(e==CIVIL_SAFETY_OFF&&!s.active&&applied.height_m==40,"separating clearance releases override after hold");
    a.height_m=10000;a.x_m=100;a.z_m=-100;a.vx_mps=0;a.vz_mps=25;
    check(civil_supervise(41,&v,&a,&c,&nominal,&s,&applied,&p)==0&&!s.active,"high altitude traffic does not force climb");
    c.safety_enabled=0;a.height_m=40;
    check(civil_supervise(42,&v,&a,&c,&nominal,&s,&applied,&p)==0&&!s.active&&applied.height_m==40,"monitor-only scenario never overrides reference");
    if(!failures)puts("PASS: safety trigger, hold, release and high-altitude/monitor handling");
}
static void integration_interface_checks(void){
    CivilConfig c;CivilState s;CivilObservation o;double old[BC_NSTATE];int e;
    civil_default(&c,CIVIL_HIGH);c.controllers[0]=c.controllers[1]=c.controllers[2]=&user_controller;
    check(civil_initial(&c,&s)==0,"civil scenario initializes real static release");
    memcpy(old,s.vehicle.a,sizeof(old));e=civil_update(0,&s,&c,&o);
    check(e>=0&&o.controller_id==101&&o.nominal.height_m==45&&s.calls[0]==1,"custom departure controller called");
    check(memcmp(old,s.vehicle.a,sizeof(old))==0,"controller update leaves continuous flight state unchanged");
    s.vehicle.a[0]=900;s.vehicle.boost_active=0;s.vehicle.cruise_ignited=1;s.vehicle.ignition_time=3;
    s.vehicle.a[15]=0;s.vehicle.a[13]=c.base.vehicle.mass_dry+s.vehicle.a[16];
    memcpy(old,s.vehicle.a,sizeof(old));e=civil_update(8,&s,&c,&o);
    check(e>=0&&s.phase==CIVIL_ENROUTE&&s.waypoint==1&&o.nominal.height_m==65&&s.calls[1]==1,"custom enroute controller and waypoint progression");
    check(memcmp(old,s.vehicle.a,sizeof(old))==0,"phase handoff preserves continuous state");
    s.vehicle.a[0]=2300;e=civil_update(9,&s,&c,&o);
    check(e>=0&&s.phase==CIVIL_APPROACH&&o.controller_id==101&&s.calls[2]==1,"custom approach controller called");
    c.controllers[2]=&bad_controller;check(civil_update(10,&s,&c,&o)<0,"invalid callback reference fails instead of using stale command");
    civil_default(&c,CIVIL_HIGH);c.waypoints[1].x_m=100;check(civil_initial(&c,&s)<0,"unordered route rejected");
    civil_default(&c,CIVIL_HIGH);c.controllers[0]=NULL;check(civil_initial(&c,&s)<0,"missing callback rejected");
    if(!failures)puts("PASS: custom callbacks in all phases, state continuity and invalid configurations");
}
int main(void){controller_checks();traffic_cpa_checks();supervisor_checks();integration_interface_checks();return failures?1:0;}
