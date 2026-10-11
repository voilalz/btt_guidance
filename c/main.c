#include "trajectory.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const double DEG=57.2957795130823208768;
static int positive_number(const char *arg,double *out) {
    char *end; errno=0; double value=strtod(arg,&end);
    if(errno || end==arg || *end!='\0' || !isfinite(value) || value<=0.0) return 0;
    *out=value; return 1;
}
static int row(FILE *file,double t,const VTState *s,const VTParams *p,VTMode mode) {
    VTObservation o; vt_observe(t,s,p,mode,&o);
    for(int j=0;j<VT_NSTATE;j++) if(!isfinite(s->a[j])) return 0;
    return fprintf(file,
        "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
        t,s->a[0],s->a[1],s->a[2],s->a[3],s->a[4],s->a[5],o.speed,
        o.theta*DEG,o.psi_v*DEG,o.alpha*DEG,o.bank*DEG,s->a[6],o.throttle,
        o.thrust_cmd,o.thrust,o.lift,o.drag,o.rho,o.fuel_flow)>0;
}
int main(int argc,char **argv) {
    if(argc<3 || argc>5) {
        fprintf(stderr,"Usage: %s constant|variable output.csv [dt [duration]]\n",argv[0]);
        return 2;
    }
    VTMode mode;
    if(strcmp(argv[1],"constant")==0) mode=VT_CONSTANT;
    else if(strcmp(argv[1],"variable")==0) mode=VT_VARIABLE;
    else { fprintf(stderr,"Invalid mode.\n"); return 2; }
    double dt=0.02,duration=120.0;
    if((argc>=4 && !positive_number(argv[3],&dt)) ||
       (argc>=5 && !positive_number(argv[4],&duration)) ||
       dt>1.0 || duration>3600.0 || duration/dt>1000000.0) {
        fprintf(stderr,"Require 0<dt<=1 s, 0<duration<=3600 s and <=1000000 steps.\n");
        return 2;
    }
    FILE *file=fopen(argv[2],"w");
    if(!file) { perror("Cannot open output CSV"); return 1; }
    fputs("t_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,speed_mps,theta_deg,psi_v_deg,alpha_deg,bank_deg,mass_kg,throttle,thrust_cmd_N,thrust_N,lift_N,drag_N,rho_kgm3,fuel_flow_kgps\n",file);
    VTParams p; VTState s; vt_default(&p); vt_initial(&p,&s);
    double t=0.0,max_height=s.a[1]; int landed=0;
    if(!row(file,t,&s,&p,mode)) { fclose(file); return 1; }
    unsigned long count=1;
    while(t<duration-duration*1e-12) {
        double h=fmin(dt,duration-t),used=0.0;
        landed=vt_advance(t,h,&s,&p,mode,&used); t+=used;
        max_height=fmax(max_height,s.a[1]);
        if(!row(file,t,&s,&p,mode)) {
            fprintf(stderr,"Nonfinite state or CSV write failure.\n"); fclose(file); return 1;
        }
        count++;
        if(landed) break;
    }
    if(fclose(file)!=0) { perror("CSV close failed"); return 1; }
    VTObservation o; vt_observe(t,&s,&p,mode,&o);
    printf("mode=%s, rows=%lu, stop=%s\n",argv[1],count,landed?"ground":"time_limit");
    printf("t=%.6f s, x=%.3f m, y=%.3f m, z=%.3f m\n",t,s.a[0],s.a[1],s.a[2]);
    printf("speed=%.6f m/s, max_height=%.3f m, fuel_used=%.9f kg\n",o.speed,max_height,p.mass0-s.a[6]);
    return 0;
}
