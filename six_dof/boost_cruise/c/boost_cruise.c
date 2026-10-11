#include "boost_cruise.h"
#include <math.h>
#include <string.h>

static const double PI=3.14159265358979323846;
static double clip(double x,double lo,double hi){return fmax(lo,fmin(hi,x));}
static int valid_params(const BCParams *p){
    int i;
    if(!p)return 0;
    {
        const double a[]={p->boost_thrust,p->burn_duration,p->boost_fuel0,p->cruise_fuel0,
            p->launch_pitch,p->launch_height,p->ignition_height,p->ignition_speed,
            p->cruise_height,p->cruise_speed,p->cruise_idle_thrust,p->transition_time,p->height_gain,
            p->gamma_limit,p->pitch_gain,p->rate_gain,p->elevator_limit,p->speed_gain};
        for(i=0;i<(int)(sizeof(a)/sizeof(a[0]));++i)if(!isfinite(a[i]))return 0;
    }
    return p->boost_thrust>0&&p->burn_duration>0&&p->boost_fuel0>0&&p->cruise_fuel0>0&&
        p->launch_height>=0&&p->launch_pitch>0&&p->launch_pitch<PI/2&&
        p->ignition_height>=0&&p->ignition_speed>0&&p->cruise_height>0&&p->cruise_speed>0&&
        p->transition_time>0&&p->height_gain>=0&&p->gamma_limit>0&&p->gamma_limit<PI/2&&
        p->pitch_gain>=0&&p->rate_gain>=0&&p->elevator_limit>0&&p->speed_gain>=0&&
        isfinite(p->boost_fuel0/p->burn_duration)&&
        isfinite(p->vehicle.thrust_max)&&p->vehicle.thrust_max>0&&
        p->cruise_idle_thrust>=0&&p->cruise_idle_thrust<=p->vehicle.thrust_max&&
        isfinite(p->vehicle.pitch_e)&&fabs(p->vehicle.pitch_e)>1e-12;
}
static int finite_state(const BCState *s){
    int i;if(!s||!isfinite(s->ignition_time))return 0;
    for(i=0;i<BC_NSTATE;++i)if(!isfinite(s->a[i]))return 0;
    return s->a[13]>0;
}
static int cruise_on(const BCState *s){return s->cruise_ignited&&!s->cruise_exhausted;}
static int ready(const BCState *s,const BCParams *p){
    return -s->a[2]>=p->ignition_height&&hypot(hypot(s->a[3],s->a[4]),s->a[5])>=p->ignition_speed;
}
static int normalize(BCState *s,const BCParams *p){
    double n=hypot(hypot(s->a[6],s->a[7]),hypot(s->a[8],s->a[9]));int i;
    if(!isfinite(n)||n<1e-12)return -1;
    for(i=6;i<10;++i)s->a[i]/=n;
    s->a[13]=p->vehicle.mass_dry+s->a[15]+s->a[16];
    return finite_state(s)?0:-1;
}
void bc_default(BCParams *p){
    memset(p,0,sizeof(*p));sd6_default(&p->vehicle);p->vehicle.disturbances=0;
    p->boost_thrust=260;p->burn_duration=4;p->boost_fuel0=.5;p->cruise_fuel0=.3;
    p->launch_pitch=35*PI/180;p->launch_height=0;
    p->ignition_height=40;p->ignition_speed=25;p->cruise_height=40;p->cruise_speed=25;
    p->cruise_idle_thrust=3;p->transition_time=3;p->height_gain=.02;p->gamma_limit=8*PI/180;
    p->pitch_gain=.8;p->rate_gain=.25;p->elevator_limit=20*PI/180;p->speed_gain=5;p->stabilize=1;
}
int bc_initial(BCParams *p,BCState *s){
    SD6Params trim;SD6State dummy;
    if(!s||!valid_params(p))return -1;
    trim=p->vehicle;trim.mass0=trim.mass_dry+p->cruise_fuel0;
    trim.speed0=p->cruise_speed;trim.height0=p->cruise_height;
    if(sd6_initial(&trim,&dummy)<0)return -1;
    p->vehicle.alpha_trim=trim.alpha_trim;p->vehicle.elevator_trim=trim.elevator_trim;
    p->vehicle.throttle_trim=trim.throttle_trim;
    memset(s,0,sizeof(*s));s->a[2]=-p->launch_height;
    s->a[6]=cos(p->launch_pitch/2);s->a[8]=sin(p->launch_pitch/2);
    s->a[15]=p->boost_fuel0;s->a[16]=p->cruise_fuel0;
    s->a[13]=p->vehicle.mass_dry+s->a[15]+s->a[16];
    s->boost_active=1;s->ignition_time=-1;s->airborne=p->launch_height>0;
    if(p->launch_height==0&&p->boost_thrust*sin(p->launch_pitch)/s->a[13]<=p->vehicle.g)return -2;
    return 0;
}
int bc_observe(double t,const BCState *s,const BCParams *p,BCObservation *o){
    SD6State flight;SD6Params v;double cmd,blend,target,eff;int on;
    if(!o||!isfinite(t)||!valid_params(p)||!finite_state(s))return -1;
    memcpy(flight.a,s->a,sizeof(flight.a));
    flight.a[13]=p->vehicle.mass_dry+s->a[15]+s->a[16];
    on=cruise_on(s);if(!on)flight.a[14]=0;
    v=p->vehicle;v.consume_fuel=0;v.disturbances=0;v.throttle_override=0;
    memset(o,0,sizeof(*o));
    if(sd6_observe(t,&flight,&v,SD6_CONSTANT,&o->flight)<0)return -1;
    o->height_gate=-s->a[2]>=p->ignition_height;
    o->speed_gate=o->flight.speed>=p->ignition_speed;
    o->boost_thrust=s->boost_active?p->boost_thrust:0;
    o->boost_flow=s->boost_active?p->boost_fuel0/p->burn_duration:0;
    o->cruise_flow=on?p->vehicle.fuel_c0+p->vehicle.fuel_cT*s->a[14]:0;
    o->total_thrust=o->boost_thrust+(on?s->a[14]:0);
    o->stage=s->boost_active?(on?BC_OVERLAP:BC_BOOST):(on?BC_CRUISE:(s->cruise_ignited?BC_EMPTY:BC_COAST));
    o->pitch_cmd=p->launch_pitch;o->gamma_cmd=0;cmd=on?p->vehicle.thrust_max*p->vehicle.throttle_trim:0;
    if(p->stabilize){
        if(s->cruise_ignited){
            o->gamma_cmd=clip(p->height_gain*(p->cruise_height+s->a[2]),-p->gamma_limit,p->gamma_limit);
            target=p->vehicle.alpha_trim+o->gamma_cmd;
            blend=clip((t-s->ignition_time)/p->transition_time,0,1);blend=blend*blend*(3-2*blend);
            o->pitch_cmd=(1-blend)*p->launch_pitch+blend*target;
        }
        eff=-(p->vehicle.pitch0+p->vehicle.pitch_alpha*o->flight.alpha)/p->vehicle.pitch_e;
        v.elevator_trim=clip(eff+p->pitch_gain*(o->flight.pitch-o->pitch_cmd)+p->rate_gain*s->a[11],
                              -p->elevator_limit,p->elevator_limit);
        if(on)cmd=(o->flight.drag+flight.a[13]*v.g*sin(o->gamma_cmd))/fmax(cos(o->flight.alpha),.8)
                    +p->speed_gain*(p->cruise_speed-o->flight.speed);
    }
    if(!isfinite(cmd)||!isfinite(v.elevator_trim)||!isfinite(o->cruise_flow))return -1;
    cmd=on?clip(cmd,p->cruise_idle_thrust,v.thrust_max):0;v.throttle_override=cmd/v.thrust_max;
    if(sd6_observe(t,&flight,&v,SD6_CONSTANT,&o->flight)<0)return -1;
    o->flight.thrust_cmd=cmd;o->flight.throttle=v.throttle_override;
    return 0;
}
int bc_rhs(double t,const BCState *s,const BCParams *p,double ds[BC_NSTATE]){
    SD6State flight,d;SD6Params v;BCObservation o;int i,on;
    if(!ds||bc_observe(t,s,p,&o)<0)return -1;
    memcpy(flight.a,s->a,sizeof(flight.a));flight.a[13]=p->vehicle.mass_dry+s->a[15]+s->a[16];
    on=cruise_on(s);if(!on)flight.a[14]=0;
    v=p->vehicle;v.consume_fuel=0;v.disturbances=0;
    v.elevator_trim=o.flight.de;v.throttle_override=o.flight.throttle;
    if(sd6_rhs(t,&flight,&v,SD6_CONSTANT,&d)<0)return -1;
    for(i=0;i<SD6_NSTATE;++i)ds[i]=d.a[i];
    ds[3]+=o.boost_thrust/flight.a[13];
    ds[13]=-o.boost_flow-o.cruise_flow;
    ds[14]=on?(o.flight.thrust_cmd-s->a[14])/v.thrust_tau:0;
    ds[15]=-o.boost_flow;ds[16]=-o.cruise_flow;
    for(i=0;i<BC_NSTATE;++i)if(!isfinite(ds[i]))return -1;
    return 0;
}
static int step(double t,double dt,const BCState *start,const BCParams *p,BCState *out){
    BCState y;double k1[BC_NSTATE],k2[BC_NSTATE],k3[BC_NSTATE],k4[BC_NSTATE];int i;
    if(dt==0){*out=*start;return 0;}
    if(!isfinite(dt)||dt<0||!isfinite(t+dt))return -1;
    if(bc_rhs(t,start,p,k1)<0)return -1;
    y=*start;for(i=0;i<BC_NSTATE;++i)y.a[i]=start->a[i]+dt*k1[i]/2;
    if(bc_rhs(t+dt/2,&y,p,k2)<0)return -1;
    for(i=0;i<BC_NSTATE;++i)y.a[i]=start->a[i]+dt*k2[i]/2;
    if(bc_rhs(t+dt/2,&y,p,k3)<0)return -1;
    for(i=0;i<BC_NSTATE;++i)y.a[i]=start->a[i]+dt*k3[i];
    if(bc_rhs(t+dt,&y,p,k4)<0)return -1;
    *out=*start;for(i=0;i<BC_NSTATE;++i)out->a[i]=start->a[i]+dt*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
    if(normalize(out,p)<0||out->a[14]<-1e-9||out->a[14]>p->vehicle.thrust_max+1e-9)return -1;
    out->a[14]=clip(out->a[14],0,p->vehicle.thrust_max);return 0;
}
static int crossed(int event,const BCState *s,const BCParams *p){
    if(event==BC_IGNITION)return ready(s,p);
    if(event==BC_FUEL_OUT)return s->a[16]<=0;
    return s->a[2]>=0;
}
static int locate(double t,double dt,const BCState *s,const BCParams *p,int event,double *time){
    double lo=0,hi=dt,mid;BCState probe;int i;
    for(i=0;i<55;++i){mid=(lo+hi)/2;if(step(t,mid,s,p,&probe)<0)return -1;
        if(crossed(event,&probe,p))hi=mid;else lo=mid;}
    *time=hi;return 0;
}
int bc_advance(double t,double dt,BCState *s,const BCParams *p,double *used_dt){
    BCState probe,out;double span,times[4],earliest;int flags=0,i;
    const int kinds[4]={BC_IGNITION,BC_BURNOUT,BC_FUEL_OUT,BC_GROUND};
    if(!used_dt)return -1;
    *used_dt=0;
    if(!isfinite(t)||t<0||!isfinite(dt)||dt<=0||!isfinite(t+dt)||!valid_params(p)||!finite_state(s)||
        s->a[15]<-1e-12||s->a[16]<-1e-12||s->a[14]<0||s->a[14]>p->vehicle.thrust_max+1e-9||
        fabs(s->a[13]-(p->vehicle.mass_dry+s->a[15]+s->a[16]))>1e-9)return -1;
    if(s->airborne&&s->a[2]>=0){s->a[2]=0;return BC_GROUND;}
    span=dt;
    for(i=0;i<4;++i)times[i]=INFINITY;
    if(s->boost_active&&p->burn_duration-t<=dt){span=fmax(p->burn_duration-t,0);times[1]=span;}
    /* An initial gate hit is an event even if a later probe would lose height. */
    if(!s->cruise_ignited&&ready(s,p)){span=0;times[0]=0;}
    if(step(t,span,s,p,&probe)<0)return -1;
    if(!s->cruise_ignited&&!isfinite(times[0])&&ready(&probe,p)&&
        locate(t,span,s,p,BC_IGNITION,&times[0])<0)return -1;
    if(cruise_on(s)&&probe.a[16]<=0){
        if(s->a[16]<=0)times[2]=0;
        else if(locate(t,span,s,p,BC_FUEL_OUT,&times[2])<0)return -1;
    }
    if(span>0&&probe.a[2]>=0){
        if(locate(t,span,s,p,BC_GROUND,&times[3])<0)return -1;
    }
    earliest=span;for(i=0;i<4;++i)earliest=fmin(earliest,times[i]);
    if(step(t,earliest,s,p,&out)<0)return -1;
    for(i=0;i<4;++i)if(isfinite(times[i])&&fabs(times[i]-earliest)<1e-11)flags|=kinds[i];
    if(flags&BC_GROUND){out.a[2]=0;flags=BC_GROUND;}
    else{
        if(flags&BC_IGNITION){out.cruise_ignited=1;out.ignition_time=t+earliest;}
        if(flags&BC_BURNOUT){out.boost_active=0;out.a[15]=0;}
        if(flags&BC_FUEL_OUT){out.cruise_exhausted=1;out.a[16]=0;out.a[14]=0;}
    }
    if(out.a[2]<0)out.airborne=1;
    if(out.a[15]<-1e-10||out.a[16]<-1e-10||normalize(&out,p)<0)return -1;
    *s=out;*used_dt=earliest;return flags;
}
