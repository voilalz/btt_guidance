#include "trajectory.h"
#include <math.h>
#include <string.h>

static const double PI=3.14159265358979323846;
static double clamp(double v,double lo,double hi) { return fmax(lo,fmin(hi,v)); }
static double dot3(const double a[3],const double b[3]) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static void cross3(const double a[3],const double b[3],double out[3]) {
    out[0]=a[1]*b[2]-a[2]*b[1];
    out[1]=a[2]*b[0]-a[0]*b[2];
    out[2]=a[0]*b[1]-a[1]*b[0];
}
static int engine_on(const VTState *s,const VTParams *p) {
    return !p->consume_fuel || s->a[6]>p->mass_dry+1e-12;
}

void vt_default(VTParams *p) {
    memset(p,0,sizeof(*p));
    p->g=9.80665; p->rho0=1.225; p->scale_height=8500.0; p->area=0.8;
    p->cl0=0.2; p->cl_alpha=4.5; p->cl_min=-0.6; p->cl_max=1.2;
    p->cd0=0.035; p->drag_k=0.06;
    p->mass0=12.0; p->mass_dry=11.7;
    p->thrust_max=60.0; p->thrust_tau=1.0;
    /* 虚构耗油关系q_f=c0+cT*T；不是任何真实发动机标定。 */
    p->fuel_c0=5e-5; p->fuel_cT=1.5e-5; p->consume_fuel=1;
    p->speed0=25.0; p->height0=200.0;
    p->alpha_amp=0.25*PI/180.0; p->alpha_period=30.0;
    p->bank_amp=8.0*PI/180.0; p->bank_period=60.0;
    p->throttle_mean=0.2; p->throttle_amp1=0.15; p->throttle_period1=40.0;
    p->throttle_amp2=0.05; p->throttle_period2=15.0;
    /* 只平衡初始竖直力：qS*CL+T*sin(alpha)=mg。
       这不是完整配平；纵向残余力仍由运动方程积分。 */
    double rho=p->rho0*exp(-p->height0/p->scale_height);
    double qs=0.5*rho*p->speed0*p->speed0*p->area;
    double thrust=p->thrust_max*p->throttle_mean;
    double alpha=(p->mass0*p->g/qs-p->cl0)/p->cl_alpha;
    for(int j=0;j<8;j++) {
        double residual=qs*(p->cl0+p->cl_alpha*alpha)+thrust*sin(alpha)-p->mass0*p->g;
        alpha-=residual/(qs*p->cl_alpha+thrust*cos(alpha));
    }
    p->alpha_trim=alpha;
}

void vt_initial(const VTParams *p,VTState *s) {
    memset(s,0,sizeof(*s));
    s->a[1]=p->height0; s->a[3]=p->speed0; s->a[6]=p->mass0;
    s->a[7]=p->thrust_max*p->throttle_mean;
    if(!engine_on(s,p)) s->a[7]=0.0;
}

static void observe_internal(double t,const VTState *s,const VTParams *p,
                             VTMode mode,int on,VTObservation *o) {
    memset(o,0,sizeof(*o));
    double horizontal=hypot(s->a[3],s->a[5]);
    o->speed=hypot(horizontal,s->a[4]);
    o->theta=atan2(s->a[4],horizontal);
    /* 与原图的dz/dt=-V*cos(theta)*sin(psi_v)保持同一符号。 */
    o->psi_v=atan2(-s->a[5],s->a[3]);
    o->alpha=p->alpha_trim+p->alpha_amp*sin(2.0*PI*t/p->alpha_period);
    o->bank=p->bank_amp*sin(2.0*PI*t/p->bank_period);
    o->rho=p->rho0*exp(-fmax(s->a[1],0.0)/p->scale_height);
    double cl=clamp(p->cl0+p->cl_alpha*o->alpha,p->cl_min,p->cl_max);
    double cd=p->cd0+p->drag_k*cl*cl;
    double qs=0.5*o->rho*o->speed*o->speed*p->area;
    o->lift=qs*cl; o->drag=qs*cd;
    o->throttle=p->throttle_mean;
    if(mode==VT_VARIABLE) {
        o->throttle+=p->throttle_amp1*sin(2.0*PI*t/p->throttle_period1)
                    +p->throttle_amp2*sin(2.0*PI*t/p->throttle_period2);
    }
    o->throttle=clamp(o->throttle,0.0,1.0);
    o->thrust_cmd=on ? p->thrust_max*o->throttle : 0.0;
    o->thrust=on ? clamp(s->a[7],0.0,p->thrust_max) : 0.0;
    o->fuel_flow=(on && p->consume_fuel) ? p->fuel_c0+p->fuel_cT*o->thrust : 0.0;
}
void vt_observe(double t,const VTState *s,const VTParams *p,VTMode mode,VTObservation *o) {
    observe_internal(t,s,p,mode,engine_on(s,p),o);
}

/* 根据速度建立升力方向。近竖直时用x轴投影代替y轴投影。
   零速时取x轴作参考方向；仅保证数学上有限，不表示真实起飞姿态。 */
static void basis(const VTState *s,double ev[3],double up[3],double side[3]) {
    double v[3]={s->a[3],s->a[4],s->a[5]};
    double speed=sqrt(dot3(v,v));
    if(speed<1e-9) { ev[0]=1.0; ev[1]=0.0; ev[2]=0.0; }
    else for(int i=0;i<3;i++) ev[i]=v[i]/speed;
    double reference[3]={0.0,1.0,0.0};
    double proj=dot3(reference,ev);
    for(int i=0;i<3;i++) up[i]=reference[i]-proj*ev[i];
    double norm=sqrt(dot3(up,up));
    if(norm<1e-8) {
        reference[0]=1.0; reference[1]=0.0;
        proj=dot3(reference,ev);
        for(int i=0;i<3;i++) up[i]=reference[i]-proj*ev[i];
        norm=sqrt(dot3(up,up));
    }
    for(int i=0;i<3;i++) up[i]/=norm;
    cross3(ev,up,side);
}

static void rhs_internal(double t,const VTState *s,const VTParams *p,
                         VTMode mode,int on,VTState *ds) {
    VTObservation o; observe_internal(t,s,p,mode,on,&o);
    double ev[3],up[3],side[3]; basis(s,ev,up,side);
    double mass=fmax(s->a[6],p->mass_dry);
    memset(ds,0,sizeof(*ds));
    for(int i=0;i<3;i++) {
        double en=cos(o.bank)*up[i]+sin(o.bank)*side[i];
        /* 推力与气动力在同一个惯性系相加：
           F=(T*cos(alpha)-D)*ev+(T*sin(alpha)+L)*en。 */
        ds->a[i]=s->a[i+3];
        ds->a[i+3]=((o.thrust*cos(o.alpha)-o.drag)*ev[i]
                  +(o.thrust*sin(o.alpha)+o.lift)*en)/mass;
    }
    ds->a[4]-=p->g;
    ds->a[6]=-o.fuel_flow;
    ds->a[7]=on ? (o.thrust_cmd-s->a[7])/p->thrust_tau : 0.0;
    /* T已是发动机净推力，不再额外加一次排气动量项。 */
}
void vt_rhs(double t,const VTState *s,const VTParams *p,VTMode mode,VTState *ds) {
    rhs_internal(t,s,p,mode,engine_on(s,p),ds);
}

static void rk4(double t,double h,const VTState *start,const VTParams *p,
                VTMode mode,int on,VTState *out) {
    VTState k1,k2,k3,k4,tmp;
    rhs_internal(t,start,p,mode,on,&k1);
    for(int j=0;j<VT_NSTATE;j++) tmp.a[j]=start->a[j]+0.5*h*k1.a[j];
    rhs_internal(t+0.5*h,&tmp,p,mode,on,&k2);
    for(int j=0;j<VT_NSTATE;j++) tmp.a[j]=start->a[j]+0.5*h*k2.a[j];
    rhs_internal(t+0.5*h,&tmp,p,mode,on,&k3);
    for(int j=0;j<VT_NSTATE;j++) tmp.a[j]=start->a[j]+h*k3.a[j];
    rhs_internal(t+h,&tmp,p,mode,on,&k4);
    for(int j=0;j<VT_NSTATE;j++)
        out->a[j]=start->a[j]+h*(k1.a[j]+2.0*k2.a[j]+2.0*k3.a[j]+k4.a[j])/6.0;
}

void vt_step(double t,double dt,VTState *s,const VTParams *p,VTMode mode) {
    VTState start=*s,next; int on=engine_on(&start,p);
    if(!on) { start.a[6]=p->mass_dry; start.a[7]=0.0; }
    rk4(t,dt,&start,p,mode,on,&next);
    if(on && p->consume_fuel && next.a[6]<=p->mass_dry) {
        /* 步内定位燃料耗尽。各RK阶段保持同一个发动机开关，
           避免阶段内提前关机造成质量在干质量附近反复振荡。 */
        double lo=0.0,hi=dt; VTState probe;
        for(int j=0;j<45;j++) {
            double mid=0.5*(lo+hi);
            rk4(t,mid,&start,p,mode,1,&probe);
            if(probe.a[6]>p->mass_dry) lo=mid; else hi=mid;
        }
        double burn_time=0.5*(lo+hi);
        rk4(t,burn_time,&start,p,mode,1,&probe);
        probe.a[6]=p->mass_dry; probe.a[7]=0.0;
        rk4(t+burn_time,dt-burn_time,&probe,p,mode,0,&next);
    }
    next.a[6]=fmax(next.a[6],p->mass_dry);
    next.a[7]=clamp(next.a[7],0.0,p->thrust_max);
    *s=next;
}

int vt_advance(double t,double dt,VTState *s,const VTParams *p,VTMode mode,double *used_dt) {
    if(s->a[1]<=0.0) { *used_dt=0.0; s->a[1]=0.0; return 1; }
    VTState start=*s; vt_step(t,dt,s,p,mode);
    *used_dt=dt;
    if(s->a[1]>0.0) return 0;
    double lo=0.0,hi=dt;
    for(int j=0;j<45;j++) {
        double mid=0.5*(lo+hi); VTState probe=start;
        vt_step(t,mid,&probe,p,mode);
        if(probe.a[1]>0.0) lo=mid; else hi=mid;
    }
    *s=start; *used_dt=0.5*(lo+hi);
    vt_step(t,*used_dt,s,p,mode);
    s->a[1]=0.0;
    return 1;
}
