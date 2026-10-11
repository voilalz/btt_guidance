#include "six_dof.h"
#include <math.h>
#include <string.h>

static const double PI=3.14159265358979323846;
static double clip(double x,double lo,double hi) { return fmax(lo,fmin(hi,x)); }
static int valid_state(const SD6State *s) {
    int i; if(!s) return 0;
    for(i=0;i<SD6_NSTATE;++i) if(!isfinite(s->a[i])) return 0;
    return s->a[13]>0;
}
static int valid_params(const SD6Params *p) {
    int i;
    if(!p)return 0;
    /* Validate before clipping: C fmin/fmax can otherwise conceal a NaN. */
    {
        const double values[]={p->g,p->rho0,p->scale_height,p->area,p->span,p->chord,
            p->jx,p->jy,p->jz,p->cl0,p->cl_alpha,p->cl_min,p->cl_max,p->cd0,p->drag_k,
            p->cy_beta,p->cy_r,p->roll_beta,p->roll_p,p->roll_r,p->roll_a,
            p->pitch0,p->pitch_alpha,p->pitch_q,p->pitch_e,p->yaw_beta,p->yaw_p,p->yaw_r,p->yaw_rudder,
            p->mass0,p->mass_dry,p->thrust_max,p->thrust_tau,p->fuel_c0,p->fuel_cT,
            p->speed0,p->height0,p->alpha_trim,p->elevator_trim,p->throttle_trim,p->throttle_override};
        for(i=0;i<(int)(sizeof(values)/sizeof(values[0]));++i)if(!isfinite(values[i]))return 0;
    }
    return isfinite(p->g) && p->g>=0 && isfinite(p->rho0) && p->rho0>=0 &&
        isfinite(p->scale_height) && p->scale_height>0 &&
        isfinite(p->area) && p->area>=0 && isfinite(p->span) && p->span>0 &&
        isfinite(p->chord) && p->chord>0 &&
        isfinite(p->jx) && p->jx>0 && isfinite(p->jy) && p->jy>0 && isfinite(p->jz) && p->jz>0 &&
        isfinite(p->mass_dry) && p->mass_dry>0 &&
        isfinite(p->thrust_max) && p->thrust_max>=0 && isfinite(p->thrust_tau) && p->thrust_tau>0 &&
        isfinite(p->fuel_c0) && p->fuel_c0>=0 && isfinite(p->fuel_cT) && p->fuel_cT>=0;
}
static int engine_running(const SD6State *s,const SD6Params *p) {
    return !p->consume_fuel || s->a[13]>p->mass_dry;
}
static double qnorm(const double *q) { return hypot(hypot(q[0],q[1]),hypot(q[2],q[3])); }
static int normalize(SD6State *s) {
    double n=qnorm(s->a+6);int i;
    if(!isfinite(n)||n<1e-12) return -1;
    for(i=6;i<10;++i)s->a[i]/=n;
    return 0;
}
void sd6_default(SD6Params *p) {
    memset(p,0,sizeof(*p));
    p->g=9.80665;p->rho0=1.225;p->scale_height=8500;
    p->area=.8;p->span=2;p->chord=.4;p->jx=1;p->jy=1.6;p->jz=2.2;
    p->cl0=.2;p->cl_alpha=4.5;p->cl_min=-.6;p->cl_max=1.2;
    p->cd0=.035;p->drag_k=.06;p->cy_beta=-.5;p->cy_r=.12;
    p->roll_beta=-.08;p->roll_p=-.45;p->roll_r=.10;p->roll_a=.12;
    p->pitch0=.03;p->pitch_alpha=-.80;p->pitch_q=-5;p->pitch_e=-1;
    p->yaw_beta=.08;p->yaw_p=-.04;p->yaw_r=-.20;p->yaw_rudder=-.08;
    p->mass0=12;p->mass_dry=11.7;p->thrust_max=60;p->thrust_tau=1;
    p->fuel_c0=5e-5;p->fuel_cT=1.5e-5;p->consume_fuel=1;
    p->speed0=25;p->height0=200;p->aero_enabled=1;p->disturbances=1;
    p->throttle_override=-1;
}
int sd6_rotation(const double q[4],double r[3][3]) {
    double n,a,b,c,d;
    if(!q||!r) return -1;
    n=qnorm(q);if(!isfinite(n)||n<1e-12)return -1;
    a=q[0]/n;b=q[1]/n;c=q[2]/n;d=q[3]/n;
    r[0][0]=a*a+b*b-c*c-d*d;r[0][1]=2*(b*c-a*d);r[0][2]=2*(b*d+a*c);
    r[1][0]=2*(b*c+a*d);r[1][1]=a*a-b*b+c*c-d*d;r[1][2]=2*(c*d-a*b);
    r[2][0]=2*(b*d-a*c);r[2][1]=2*(c*d+a*b);r[2][2]=a*a-b*b-c*c+d*d;
    return 0;
}
static double window(double t,double t1,double t2) {
    double v;if(t<=t1||t>=t2)return 0;v=sin(PI*(t-t1)/(t2-t1));return v*v;
}
static int observe_impl(double t,const SD6State *s,const SD6Params *p,SD6Mode mode,
                        int engine_on,SD6Observation *o) {
    double R[3][3],n,a,b,c,d,u,v,w,ca,sa,cb,sb,qbar,cl,cd,cy,ph,qh,rh;
    double ex[3],ey[3],ez[3];int i,k;
    if(!o||!isfinite(t)||!valid_state(s)||!valid_params(p)||
       (mode!=SD6_CONSTANT&&mode!=SD6_VARIABLE)||sd6_rotation(s->a+6,R)<0)return -1;
    memset(o,0,sizeof(*o));n=qnorm(s->a+6);o->quaternion_error=fabs(n-1);
    a=s->a[6]/n;b=s->a[7]/n;c=s->a[8]/n;d=s->a[9]/n;
    o->roll=atan2(2*(a*b+c*d),1-2*(b*b+c*c));
    o->pitch=asin(clip(2*(a*c-d*b),-1,1));
    o->yaw=atan2(2*(a*d+b*c),1-2*(c*c+d*d));
    u=s->a[3];v=s->a[4];w=s->a[5];o->speed=hypot(hypot(u,v),w);
    for(i=0;i<3;++i)for(k=0;k<3;++k)o->vn[i]+=R[i][k]*s->a[3+k];
    o->flight_path=atan2(-o->vn[2],hypot(o->vn[0],o->vn[1]));
    o->psi_v=atan2(-o->vn[1],o->vn[0]);
    o->rho=p->rho0*exp(-fmax(-s->a[2],0)/p->scale_height);
    o->de=p->elevator_trim;
    if(p->disturbances) {
        o->da=.2*PI/180*window(t,20,40)*sin(2*PI*(t-20)/8);
        o->dr=.1*PI/180*window(t,45,65)*sin(2*PI*(t-45)/10);
    }
    o->throttle=p->throttle_trim;
    if(p->throttle_override>=0)o->throttle=p->throttle_override;
    else if(mode==SD6_VARIABLE)o->throttle+=.15*sin(2*PI*t/40)+.05*sin(2*PI*t/15);
    if(!isfinite(o->throttle))return -1;
    o->throttle=clip(o->throttle,0,1);
    o->thrust_cmd=engine_on?p->thrust_max*o->throttle:0;
    o->fuel_flow=engine_on&&p->consume_fuel?p->fuel_c0+p->fuel_cT*s->a[14]:0;
    if(o->speed>=1e-8) {
        o->alpha=atan2(w,u);o->beta=atan2(v,hypot(u,w));
        if(p->aero_enabled) {
            qbar=.5*o->rho*o->speed*o->speed;
            cl=clip(p->cl0+p->cl_alpha*o->alpha,p->cl_min,p->cl_max);
            cd=p->cd0+p->drag_k*cl*cl;cy=p->cy_beta*o->beta+p->cy_r*o->dr;
            o->lift=qbar*p->area*cl;o->drag=qbar*p->area*cd;o->side=qbar*p->area*cy;
            ca=cos(o->alpha);sa=sin(o->alpha);cb=cos(o->beta);sb=sin(o->beta);
            ex[0]=ca*cb;ex[1]=sb;ex[2]=sa*cb;
            ey[0]=-ca*sb;ey[1]=cb;ey[2]=-sa*sb;
            ez[0]=-sa;ez[1]=0;ez[2]=ca;
            for(i=0;i<3;++i)o->force[i]=-o->drag*ex[i]+o->side*ey[i]-o->lift*ez[i];
            ph=s->a[10]*p->span/(2*fmax(o->speed,1e-3));
            qh=s->a[11]*p->chord/(2*fmax(o->speed,1e-3));
            rh=s->a[12]*p->span/(2*fmax(o->speed,1e-3));
            o->moment[0]=qbar*p->area*p->span*(p->roll_beta*o->beta+p->roll_p*ph+p->roll_r*rh+p->roll_a*o->da);
            o->moment[1]=qbar*p->area*p->chord*(p->pitch0+p->pitch_alpha*o->alpha+p->pitch_q*qh+p->pitch_e*o->de);
            o->moment[2]=qbar*p->area*p->span*(p->yaw_beta*o->beta+p->yaw_p*ph+p->yaw_r*rh+p->yaw_rudder*o->dr);
        }
    }
    for(i=0;i<3;++i)if(!isfinite(o->force[i])||!isfinite(o->moment[i])||!isfinite(o->vn[i]))return -1;
    return 0;
}
int sd6_observe(double t,const SD6State *s,const SD6Params *p,SD6Mode mode,SD6Observation *o) {
    if(!valid_state(s)||!valid_params(p))return -1;
    return observe_impl(t,s,p,mode,engine_running(s,p),o);
}
static double trim_residual(double alpha,const SD6Params *p) {
    double density=p->rho0*exp(-fmax(p->height0,0)/p->scale_height);
    double qs=.5*density*p->speed0*p->speed0*p->area;
    double cl=clip(p->cl0+p->cl_alpha*alpha,p->cl_min,p->cl_max);
    return qs*cl+qs*(p->cd0+p->drag_k*cl*cl)*tan(alpha)-p->mass0*p->g;
}
int sd6_initial(SD6Params *p,SD6State *s) {
    double lo=-15*PI/180,hi=15*PI/180,flo,fhi,mid,fm,density,cl,drag,thrust;int i;
    if(!s||!valid_params(p)||!isfinite(p->mass0)||p->mass0<=p->mass_dry||
       !isfinite(p->speed0)||p->speed0<=0||!isfinite(p->height0)||p->height0<=0||
       !p->aero_enabled||p->thrust_max<=0||!isfinite(p->pitch_e)||fabs(p->pitch_e)<1e-12)return -1;
    flo=trim_residual(lo,p);fhi=trim_residual(hi,p);
    if(!isfinite(flo)||!isfinite(fhi)||flo*fhi>0)return -2;
    for(i=0;i<80;++i){
        mid=(lo+hi)/2;fm=trim_residual(mid,p);
        if(!isfinite(fm))return -2;
        if(fabs(fm)<1e-13)break;
        if((fm>0)==(flo>0)){lo=mid;flo=fm;}else hi=mid;
    }
    p->alpha_trim=mid;
    density=p->rho0*exp(-p->height0/p->scale_height);
    cl=clip(p->cl0+p->cl_alpha*mid,p->cl_min,p->cl_max);
    drag=.5*density*p->speed0*p->speed0*p->area*(p->cd0+p->drag_k*cl*cl);
    if(fabs(cos(mid))<1e-6)return -2;
    thrust=drag/cos(mid);if(!isfinite(thrust)||thrust<0||thrust>p->thrust_max)return -2;
    p->throttle_trim=thrust/p->thrust_max;
    p->elevator_trim=-(p->pitch0+p->pitch_alpha*mid)/p->pitch_e;
    if(!isfinite(p->elevator_trim)||!isfinite(p->throttle_trim))return -2;
    memset(s,0,sizeof(*s));s->a[2]=-p->height0;
    s->a[3]=p->speed0*cos(mid);s->a[5]=p->speed0*sin(mid);
    s->a[6]=cos(mid/2);s->a[8]=sin(mid/2);s->a[13]=p->mass0;s->a[14]=thrust;
    return valid_state(s)?0:-2;
}
static int rhs_impl(double t,const SD6State *s,const SD6Params *p,SD6Mode mode,
                    int engine_on,SD6State *ds) {
    SD6Observation o;double R[3][3],a,b,c,d,u,v,w,wp,wq,wr;int i;
    if(!ds||observe_impl(t,s,p,mode,engine_on,&o)<0||sd6_rotation(s->a+6,R)<0)return -1;
    memset(ds,0,sizeof(*ds));for(i=0;i<3;++i){ds->a[i]=o.vn[i];ds->a[3+i]=o.force[i]/s->a[13]+R[2][i]*p->g;}
    if(engine_on)ds->a[3]+=s->a[14]/s->a[13];
    u=s->a[3];v=s->a[4];w=s->a[5];wp=s->a[10];wq=s->a[11];wr=s->a[12];
    ds->a[3]+=wr*v-wq*w;ds->a[4]+=wp*w-wr*u;ds->a[5]+=wq*u-wp*v;
    /* Raw stage quaternion for kinematics; normalized copy only for rotation. */
    a=s->a[6];b=s->a[7];c=s->a[8];d=s->a[9];
    ds->a[6]=-.5*(b*wp+c*wq+d*wr);
    ds->a[7]=.5*(a*wp+c*wr-d*wq);
    ds->a[8]=.5*(a*wq+d*wp-b*wr);
    ds->a[9]=.5*(a*wr+b*wq-c*wp);
    ds->a[10]=(o.moment[0]+(p->jy-p->jz)*wq*wr)/p->jx;
    ds->a[11]=(o.moment[1]+(p->jz-p->jx)*wp*wr)/p->jy;
    ds->a[12]=(o.moment[2]+(p->jx-p->jy)*wp*wq)/p->jz;
    if(engine_on){ds->a[13]=-o.fuel_flow;ds->a[14]=(o.thrust_cmd-s->a[14])/p->thrust_tau;}
    for(i=0;i<SD6_NSTATE;++i)if(!isfinite(ds->a[i]))return -1;
    return 0;
}
int sd6_rhs(double t,const SD6State *s,const SD6Params *p,SD6Mode mode,SD6State *ds) {
    if(!valid_state(s)||!valid_params(p))return -1;
    return rhs_impl(t,s,p,mode,engine_running(s,p),ds);
}
int sd6_step(double t,double dt,SD6State *s,const SD6Params *p,SD6Mode mode,int engine_on) {
    SD6State y,k1,k2,k3,k4,out;int i;
    if(!valid_state(s)||!valid_params(p)||!isfinite(t)||!isfinite(dt)||dt<=0||!isfinite(t+dt))return -1;
    if(rhs_impl(t,s,p,mode,engine_on,&k1)<0)return -1;
    for(i=0;i<SD6_NSTATE;++i)y.a[i]=s->a[i]+dt*.5*k1.a[i];
    if(rhs_impl(t+dt*.5,&y,p,mode,engine_on,&k2)<0)return -1;
    for(i=0;i<SD6_NSTATE;++i)y.a[i]=s->a[i]+dt*.5*k2.a[i];
    if(rhs_impl(t+dt*.5,&y,p,mode,engine_on,&k3)<0)return -1;
    for(i=0;i<SD6_NSTATE;++i)y.a[i]=s->a[i]+dt*k3.a[i];
    if(rhs_impl(t+dt,&y,p,mode,engine_on,&k4)<0)return -1;
    for(i=0;i<SD6_NSTATE;++i)out.a[i]=s->a[i]+dt*(k1.a[i]+2*k2.a[i]+2*k3.a[i]+k4.a[i])/6;
    if(!valid_state(&out)||normalize(&out)<0||out.a[14]<-1e-9||out.a[14]>p->thrust_max+1e-9)return -1;
    out.a[14]=clip(out.a[14],0,p->thrust_max);*s=out;return 0;
}
static int fuel_step(double t,double dt,const SD6State *start,const SD6Params *p,
                     SD6Mode mode,SD6State *out) {
    SD6State probe;double lo,hi,mid;int on=engine_running(start,p),i;
    *out=*start;
    if(!on){out->a[13]=p->mass_dry;out->a[14]=0;}
    if(sd6_step(t,dt,out,p,mode,on)<0)return -1;
    if(!on||!p->consume_fuel||out->a[13]>p->mass_dry)return 0;
    /* Each probe uses a fixed propulsion mode; RK stages cannot switch it early. */
    lo=0;hi=dt;
    for(i=0;i<60;++i){
        mid=(lo+hi)/2;probe=*start;
        if(sd6_step(t,mid,&probe,p,mode,1)<0)return -1;
        if(probe.a[13]>p->mass_dry)lo=mid;else hi=mid;
    }
    *out=*start;
    if(hi>0&&sd6_step(t,hi,out,p,mode,1)<0)return -1;
    out->a[13]=p->mass_dry;out->a[14]=0;
    if(dt>hi&&sd6_step(t+hi,dt-hi,out,p,mode,0)<0)return -1;
    return 0;
}
int sd6_advance(double t,double dt,SD6State *s,const SD6Params *p,SD6Mode mode,double *used_dt) {
    SD6State out,probe;double lo,hi,mid;int i;
    if(!used_dt)return -1;
    *used_dt=0;
    if(!valid_state(s)||!valid_params(p)||!isfinite(t)||!isfinite(dt)||dt<=0||!isfinite(t+dt)||
       s->a[13]<p->mass_dry-1e-10||s->a[14]<0||s->a[14]>p->thrust_max+1e-9||
       (mode!=SD6_CONSTANT&&mode!=SD6_VARIABLE))return -1;
    if(normalize(s)<0)return -1;
    if(s->a[13]<=p->mass_dry&&p->consume_fuel){s->a[13]=p->mass_dry;s->a[14]=0;}
    if(s->a[2]>=0){s->a[2]=0;return 1;}
    if(fuel_step(t,dt,s,p,mode,&out)<0)return -1;
    if(out.a[2]>=0){
        lo=0;hi=dt;
        for(i=0;i<60;++i){
            mid=(lo+hi)/2;
            if(fuel_step(t,mid,s,p,mode,&probe)<0)return -1;
            if(probe.a[2]<0)lo=mid;else hi=mid;
        }
        if(fuel_step(t,hi,s,p,mode,&out)<0)return -1;
        out.a[2]=0;*used_dt=hi;*s=out;return 1;
    }
    if(out.a[13]<p->mass_dry-1e-10)return -1;
    *s=out;*used_dt=dt;return 0;
}
