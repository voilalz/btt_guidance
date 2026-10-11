#include "six_dof.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures;
static void check(int ok,const char *name){if(!ok){++failures;fprintf(stderr,"FAIL: %s\n",name);}}
static void base(SD6Params *p,SD6State *s){
    sd6_default(p);memset(s,0,sizeof(*s));s->a[6]=1;s->a[2]=-10;
    s->a[13]=12;p->aero_enabled=0;p->disturbances=0;p->g=0;
    p->fuel_c0=.1;p->fuel_cT=0;p->mass_dry=11.9;
    p->throttle_override=.5;s->a[14]=30;
}
static void dry_test(void){
    SD6Params p;SD6State s;double used;int rc;
    base(&p,&s);rc=sd6_advance(0,2,&s,&p,SD6_CONSTANT,&used);
    check(rc==0&&fabs(used-2)<1e-14,"dry event does not terminate flight");
    check(s.a[13]==p.mass_dry&&s.a[14]==0,"dry event mass floor and instant thrust shutdown");
    check(fabs(s.a[3]-300*log(12/11.9))<1e-8,"analytic velocity with constant thrust and mass flow until dry");
    check(sd6_advance(2,.5,&s,&p,SD6_CONSTANT,&used)==0&&s.a[13]==p.mass_dry&&s.a[14]==0,"dry state remains off");
}
static double run_ground(double dt,int dry){
    SD6Params p;SD6State s;double t=0,used,exact=sqrt(20/9.80665);int rc=0;
    base(&p,&s);p.g=9.80665;p.thrust_max=0;s.a[14]=0;
    if(!dry)p.consume_fuel=0;
    while(rc==0&&t<10){rc=sd6_advance(t,dt,&s,&p,SD6_CONSTANT,&used);t+=used;}
    check(rc==1&&s.a[2]==0,"ground event terminates at zero height");
    check(fabs(t-exact)<2e-10,"analytic ground time");
    check(fabs(s.a[5]-p.g*exact)<2e-9,"ground velocity at located event");
    if(dry)check(s.a[13]==p.mass_dry&&s.a[14]==0,"dry and ground in same step");
    return t;
}
int main(void){
    SD6Params p;SD6State s;double used,a,b;
    dry_test();a=run_ground(2,1);b=run_ground(.01,1);run_ground(.05,0);
    check(fabs(a-b)<2e-10,"event time independent of step size");
    base(&p,&s);check(sd6_advance(0,0,&s,&p,SD6_CONSTANT,&used)<0,"reject zero step");
    s.a[6]=NAN;check(sd6_advance(0,.1,&s,&p,SD6_CONSTANT,&used)<0,"reject nonfinite state");
    base(&p,&s);s.a[13]=11;check(sd6_advance(0,.1,&s,&p,SD6_CONSTANT,&used)<0,"reject mass below dry mass");
    base(&p,&s);s.a[2]=0;check(sd6_advance(0,.1,&s,&p,SD6_CONSTANT,&used)==1&&used==0,"initial ground needs no integration");
    if(!failures)puts("PASS: dry/ground analytic events, two events per step, invalid states");
    return failures?1:0;
}
