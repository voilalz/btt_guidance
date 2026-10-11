#include "boost_cruise.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NCOL 58
typedef struct { double v[NCOL]; } Row;
static int positive(const char *text,double *v){
    char *end;errno=0;*v=strtod(text,&end);
    return end!=text&&*end=='\0'&&!errno&&isfinite(*v)&&*v>0;
}
static int sample(Row *row,double t,const BCState *s,const BCParams *p,int event){
    BCObservation b;SD6Observation o;double *v=row->v;int n=0,i;
    if(bc_observe(t,s,p,&b)<0)return -1;
    o=b.flight;
    v[n++]=t;v[n++]=s->a[0];v[n++]=-s->a[2];v[n++]=s->a[1];
    v[n++]=o.vn[0];v[n++]=-o.vn[2];v[n++]=o.vn[1];
    for(i=0;i<13;++i)v[n++]=s->a[i];
    v[n++]=o.roll;v[n++]=o.pitch;v[n++]=o.yaw;v[n++]=o.alpha;v[n++]=o.beta;
    v[n++]=o.flight_path;v[n++]=o.psi_v;v[n++]=s->a[13];v[n++]=s->a[14];
    v[n++]=o.thrust_cmd;v[n++]=o.throttle;v[n++]=o.da;v[n++]=o.de;v[n++]=o.dr;
    v[n++]=o.lift;v[n++]=o.drag;v[n++]=o.side;
    for(i=0;i<3;++i)v[n++]=o.moment[i];
    v[n++]=o.quaternion_error;v[n++]=o.speed;v[n++]=b.boost_thrust;v[n++]=b.total_thrust;
    v[n++]=s->a[15];v[n++]=s->a[16];v[n++]=b.boost_flow;v[n++]=b.cruise_flow;
    v[n++]=s->boost_active;v[n++]=s->cruise_ignited;v[n++]=s->cruise_exhausted;
    v[n++]=b.stage;v[n++]=s->ignition_time;v[n++]=b.pitch_cmd;v[n++]=b.gamma_cmd;
    v[n++]=b.height_gate;v[n++]=b.speed_gate;v[n++]=event;
    return n==NCOL?0:-1;
}
static int append(Row *rows,size_t *count,size_t cap,double t,const BCState *s,const BCParams *p,int event){
    if(*count&&rows[*count-1].v[0]==t){
        event|=(int)rows[*count-1].v[NCOL-1];return sample(&rows[*count-1],t,s,p,event);
    }
    if(*count>=cap)return -1;
    if(sample(&rows[*count],t,s,p,event)<0)return -1;
    ++*count;return 0;
}
int main(int argc,char **argv){
    BCParams p;BCState s;Row *rows;FILE *f;double duration=120,dt=.005,t=0,next,used;
    unsigned long stride=10;size_t cap,count=0,step=0,i;char *end;int event=0,status=0,j;
    if(argc<2||argc>5){fprintf(stderr,"Usage: %s OUTPUT.csv [duration_s [dt_s [output_stride]]]\n",argv[0]);return 2;}
    if((argc>2&&!positive(argv[2],&duration))||(argc>3&&!positive(argv[3],&dt))||duration>3600||ceil(duration/dt)>2000000){
        fprintf(stderr,"Finite positive duration/dt required; duration <=3600 s, <=2000000 steps\n");return 2;
    }
    if(argc>4){errno=0;stride=strtoul(argv[4],&end,10);if(errno||end==argv[4]||*end||stride==0||argv[4][0]=='-'){fprintf(stderr,"Positive integer output stride required\n");return 2;}}
    bc_default(&p);if(bc_initial(&p,&s)<0){fprintf(stderr,"Invalid launch/trim configuration or insufficient upward acceleration\n");return 1;}
    f=fopen(argv[1],"w");if(!f){perror("Output CSV");return 1;}
    cap=(size_t)(ceil(duration/dt)/(double)stride)+10;rows=calloc(cap,sizeof(*rows));
    if(!rows){fclose(f);fprintf(stderr,"Cannot allocate output buffer\n");return 1;}
    status=append(rows,&count,cap,0,&s,&p,0)<0;
    while(!status&&t<duration&&!(event&BC_GROUND)){
        next=fmin(duration,(double)(step+1)*dt);
        if(next<=t){status=1;break;}
        while(!status&&t<next&&!(event&BC_GROUND)){
            event=bc_advance(t,next-t,&s,&p,&used);
            if(event<0||(used==0&&event==0)){fprintf(stderr,"Numerical/state failure at %.17g s\n",t);status=1;break;}
            t=event?t+used:next;
            if(event&&append(rows,&count,cap,t,&s,&p,event)<0)status=1;
        }
        ++step;
        if(!status&&(step%stride==0||t==duration||(event&BC_GROUND)))
            status=append(rows,&count,cap,t,&s,&p,event)<0;
    }
    if(!status){
        if(fputs("t_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,N_m,E_m,D_m,u_mps,v_mps,w_mps,quat_a,quat_b,quat_c,quat_d,p_radps,q_radps,r_radps,roll_rad,pitch_rad,yaw_rad,alpha_rad,beta_rad,flight_path_rad,psi_v_rad,mass_kg,thrust_N,thrust_cmd_N,throttle,delta_a_rad,delta_e_rad,delta_r_rad,lift_N,drag_N,side_N,Mx_Nm,My_Nm,Mz_Nm,quat_norm_error,speed_mps,boost_thrust_N,total_thrust_N,boost_fuel_kg,cruise_fuel_kg,boost_flow_kgps,cruise_flow_kgps,boost_active,cruise_ignited,cruise_exhausted,stage,ignition_time_s,pitch_cmd_rad,gamma_cmd_rad,height_gate,speed_gate,event_mask\n",f)==EOF)status=1;
        for(i=0;i<count&&!status;++i){
            for(j=0;j<NCOL;++j)if(fprintf(f,j?",%.17g":"%.17g",rows[i].v[j])<0)status=1;
            if(fputc('\n',f)==EOF)status=1;
        }
    }
    if(fclose(f)!=0)status=1;
    free(rows);
    if(!status)printf("boost/cruise: %lu samples, end=%.12g s (%s), ignition=%.12g s, height=%.9g m, speed=%.9g m/s\n",
        (unsigned long)count,t,event&BC_GROUND?"ground":"duration",s.ignition_time,-s.a[2],hypot(hypot(s.a[3],s.a[4]),s.a[5]));
    return status;
}
