#include "six_dof.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int positive(const char *text,double *v) {
    char *end;errno=0;*v=strtod(text,&end);
    return end!=text&&*end=='\0'&&errno==0&&isfinite(*v)&&*v>0;
}
static int sample(FILE *f,double t,const SD6State *s,const SD6Params *p,SD6Mode mode) {
    SD6Observation o;double v[41];int n=0,i;
    if(sd6_observe(t,s,p,mode,&o)<0)return -1;
    v[n++]=t;v[n++]=s->a[0];v[n++]=-s->a[2];v[n++]=s->a[1];
    v[n++]=o.vn[0];v[n++]=-o.vn[2];v[n++]=o.vn[1];
    for(i=0;i<3;++i)v[n++]=s->a[i];
    for(i=3;i<13;++i)v[n++]=s->a[i];
    v[n++]=o.roll;v[n++]=o.pitch;v[n++]=o.yaw;v[n++]=o.alpha;v[n++]=o.beta;
    v[n++]=o.flight_path;v[n++]=o.psi_v;v[n++]=s->a[13];v[n++]=s->a[14];
    v[n++]=o.thrust_cmd;v[n++]=o.throttle;v[n++]=o.da;v[n++]=o.de;v[n++]=o.dr;
    v[n++]=o.lift;v[n++]=o.drag;v[n++]=o.side;
    for(i=0;i<3;++i)v[n++]=o.moment[i];
    v[n++]=o.quaternion_error;
    if(n!=41)return -1;
    for(i=0;i<n;++i)if(fprintf(f,i?",%.17g":"%.17g",v[i])<0)return -1;
    return fputc('\n',f)==EOF?-1:0;
}
int main(int argc,char **argv) {
    SD6Params p;SD6State s;SD6Mode mode;FILE *f;double duration=120,dt=.01,t=0,next,used,last=0;
    unsigned long stride=5;unsigned long long steps=0,rows=1;char *end;int rc=0,status=0;
    if(argc<3||argc>6){fprintf(stderr,"Usage: %s constant|variable OUTPUT.csv [duration_s [dt_s [output_stride]]]\n",argv[0]);return 2;}
    if(strcmp(argv[1],"constant")==0)mode=SD6_CONSTANT;
    else if(strcmp(argv[1],"variable")==0)mode=SD6_VARIABLE;
    else {fprintf(stderr,"Invalid mode\n");return 2;}
    if((argc>3&&!positive(argv[3],&duration))||(argc>4&&!positive(argv[4],&dt))){fprintf(stderr,"Duration and dt must be finite and positive\n");return 2;}
    if(argc>5){errno=0;stride=strtoul(argv[5],&end,10);if(errno||end==argv[5]||*end||argv[5][0]=='-'||stride==0){fprintf(stderr,"Output stride must be a positive integer\n");return 2;}}
    sd6_default(&p);if(sd6_initial(&p,&s)<0){fprintf(stderr,"No admissible initial force/moment trim\n");return 1;}
    f=fopen(argv[2],"w");if(!f){perror("Output CSV");return 1;}
    if(fputs("t_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,N_m,E_m,D_m,u_mps,v_mps,w_mps,quat_a,quat_b,quat_c,quat_d,p_radps,q_radps,r_radps,roll_rad,pitch_rad,yaw_rad,alpha_rad,beta_rad,flight_path_rad,psi_v_rad,mass_kg,thrust_N,thrust_cmd_N,throttle,delta_a_rad,delta_e_rad,delta_r_rad,lift_N,drag_N,side_N,Mx_Nm,My_Nm,Mz_Nm,quat_norm_error\n",f)==EOF||sample(f,0,&s,&p,mode)<0)status=1;
    while(!status&&t<duration&&rc==0){
        next=fmin(duration,(double)(steps+1)*dt);
        if(next<=t){fprintf(stderr,"Time step cannot advance floating-point time\n");status=1;break;}
        rc=sd6_advance(t,next-t,&s,&p,mode,&used);
        if(rc<0){fprintf(stderr,"Numerical/state failure at t=%.17g; reduce dt or check parameters\n",t);status=1;break;}
        t=rc==1?t+used:next;++steps;
        if(steps%stride==0||rc==1||t==duration){
            if(t>last){if(sample(f,t,&s,&p,mode)<0){status=1;break;}last=t;++rows;}
        }
    }
    if(fclose(f)!=0){perror("Closing CSV");status=1;}
    if(!status)printf("%s: %llu samples, end=%.12g s (%s), mass=%.9g kg; trim alpha=%.9g rad, elevator=%.9g rad, thrust=%.9g N\n",argv[1],rows,t,rc==1?"ground":"duration",s.a[13],p.alpha_trim,p.elevator_trim,p.throttle_trim*p.thrust_max);
    return status;
}
