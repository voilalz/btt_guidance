#include "civil_route.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *headers[]={
    "t_s","x_m","height_m","z_m","vx_mps","vh_mps","vz_mps","speed_mps","mass_kg",
    "boost_thrust_N","cruise_thrust_N","cruise_cmd_N","total_thrust_N","boost_fuel_kg","cruise_fuel_kg",
    "quat_a","quat_b","quat_c","quat_d","quat_norm_error","p_radps","q_radps","r_radps",
    "roll_rad","pitch_rad","yaw_rad","alpha_rad","beta_rad","delta_e_rad","pitch_cmd_rad","gamma_cmd_rad",
    "route_height_m","route_speed_mps","command_height_m","command_speed_mps","phase","controller_id",
    "waypoint","waypoint_x_m","x_remaining_m","altitude_error_m","traffic_x_m","traffic_height_m","traffic_z_m",
    "traffic_vx_mps","traffic_vh_mps","traffic_vz_mps","separation_m","horizontal_cpa_m","tcpa_s","vertical_separation_m",
    "safety_active","safety_activations","route_complete","event_mask","boost_active","cruise_ignited","ignition_time_s","propulsion_stage"};
enum { NCOL=sizeof(headers)/sizeof(headers[0]), EVENT_COL=54 };
typedef struct { double v[NCOL]; } Row;
typedef struct { FILE *file;Row pending;int has_pending;unsigned long samples; } Csv;
static int positive(const char *text,double *out){
    char *end;errno=0;*out=strtod(text,&end);return end!=text&&*end=='\0'&&!errno&&isfinite(*out)&&*out>0;
}
static int sample(Row *row,double t,const CivilState *s,const CivilObservation *o,int event){
    const BCState *b=&s->vehicle;const BCObservation *f=&o->flight;const CivilFlightView *v=&o->own;
    double values[]={t,v->x_m,v->height_m,v->z_m,v->vx_mps,v->vh_mps,v->vz_mps,v->speed_mps,v->mass_kg,
        f->boost_thrust,b->a[14],f->flight.thrust_cmd,f->total_thrust,b->a[15],b->a[16],
        b->a[6],b->a[7],b->a[8],b->a[9],f->flight.quaternion_error,b->a[10],b->a[11],b->a[12],
        f->flight.roll,f->flight.pitch,f->flight.yaw,f->flight.alpha,f->flight.beta,f->flight.de,f->pitch_cmd,f->gamma_cmd,
        o->nominal.height_m,o->nominal.speed_mps,o->applied.height_m,o->applied.speed_mps,o->phase,o->controller_id,
        o->waypoint,o->segment.x_m,o->segment.x_m-v->x_m,v->height_m-o->nominal.height_m,
        o->traffic.x_m,o->traffic.height_m,o->traffic.z_m,o->traffic.vx_mps,o->traffic.vh_mps,o->traffic.vz_mps,
        o->prediction.separation_m,o->prediction.horizontal_m,o->prediction.tcpa_s,o->prediction.vertical_m,
        s->safety.active,s->safety.activations,s->route_complete,event,b->boost_active,b->cruise_ignited,b->ignition_time,f->stage};
    int i;if(sizeof(values)/sizeof(values[0])!=NCOL)return -1;
    for(i=0;i<NCOL;++i)if(!isfinite(values[i]))return -1;
    memcpy(row->v,values,sizeof(values));return 0;
}
static int flush(Csv *csv){
    int i;if(!csv->has_pending)return 0;
    for(i=0;i<NCOL;++i)if(fprintf(csv->file,i?",%.17g":"%.17g",csv->pending.v[i])<0)return -1;
    if(fputc('\n',csv->file)==EOF)return -1;
    ++csv->samples;csv->has_pending=0;return 0;
}
static int append(Csv *csv,double t,const CivilState *s,const CivilObservation *o,int event){
    if(csv->has_pending&&csv->pending.v[0]==t)event|=(int)csv->pending.v[EVENT_COL];
    else if(csv->has_pending&&(t<csv->pending.v[0]||flush(csv)<0))return -1;
    if(sample(&csv->pending,t,s,o,event)<0)return -1;
    csv->has_pending=1;return 0;
}
static int write_summary(const char *path,const char *scene,const CivilConfig *c,const CivilState *s,
                         const CivilObservation *o,double t,double dt,double min_sep,double min_time,double max_q,
                         unsigned long stride,unsigned long steps,unsigned long samples,int ground){
    FILE *f=fopen(path,"w");int rc;if(!f)return -1;
    rc=fprintf(f,"{\n  \"scene\": \"%s\",\n  \"end_time_s\": %.17g,\n  \"termination\": \"%s\",\n"
        "  \"dt_s\": %.17g,\n  \"output_stride\": %lu,\n"
        "  \"controllers\": [\"%s\", \"%s\", \"%s\"],\n  \"controller_calls\": [%lu, %lu, %lu],\n"
        "  \"accepted_steps\": %lu,\n  \"samples\": %lu,\n  \"minimum_separation_m\": %.17g,\n"
        "  \"minimum_separation_time_s\": %.17g,\n  \"max_quaternion_norm_error\": %.17g,\n"
        "  \"safety_activations\": %d,\n  \"route_complete\": %d,\n  \"traffic_height_m\": %.17g,\n"
        "  \"ignition_time_s\": %.17g,\n  \"final_position_m\": [%.17g, %.17g, %.17g],\n"
        "  \"final_speed_mps\": %.17g,\n  \"final_mass_kg\": %.17g\n}\n",
        scene,t,ground?"ground":"duration",dt,stride,c->controllers[0]->name,c->controllers[1]->name,c->controllers[2]->name,
        s->calls[0],s->calls[1],s->calls[2],steps,samples,min_sep,min_time,max_q,s->safety.activations,s->route_complete,
        c->traffic0.height_m,s->vehicle.ignition_time,o->own.x_m,o->own.height_m,o->own.z_m,o->own.speed_mps,o->own.mass_kg);
    if(fclose(f)!=0)return -1;
    return rc<0?-1:0;
}
int main(int argc,char **argv){
    CivilConfig c;CivilState s;CivilObservation o;Csv csv={0};
    double duration=120,dt=.005,t=0,next,used,min_sep,min_time=0,max_q=0;
    unsigned long stride=10,step=0,accepted=0;int scene=-1,event,status=0,i,logical;
    char *end,jsonpath[4096];
    if(argc<3||argc>9){fprintf(stderr,"Usage: %s high|crossing|monitor OUTPUT.csv [duration [dt [stride [departure [enroute [approach]]]]]]\n",argv[0]);return 2;}
    if(strcmp(argv[1],"high")==0)scene=CIVIL_HIGH;
    if(strcmp(argv[1],"crossing")==0)scene=CIVIL_CROSSING;
    if(strcmp(argv[1],"monitor")==0)scene=CIVIL_MONITOR;
    if(scene<0||(argc>3&&!positive(argv[3],&duration))||(argc>4&&!positive(argv[4],&dt))||
        duration>3600||ceil(duration/dt)>2000000){fprintf(stderr,"Valid scene and finite positive duration/dt required; <=3600 s and <=2000000 steps\n");return 2;}
    if(argc>5){errno=0;stride=strtoul(argv[5],&end,10);if(errno||end==argv[5]||*end||stride==0||argv[5][0]=='-'){fprintf(stderr,"Positive integer output stride required\n");return 2;}}
    civil_default(&c,scene);
    for(i=0;i<3;++i)if(argc>6+i){c.controllers[i]=civil_controller(argv[6+i]);if(!c.controllers[i]){fprintf(stderr,"Unknown controller: %s\n",argv[6+i]);return 2;}}
    if(strlen(argv[2])+6>sizeof(jsonpath)){fprintf(stderr,"Output path too long\n");return 2;}
    snprintf(jsonpath,sizeof(jsonpath),"%s.json",argv[2]);
    if(civil_initial(&c,&s)<0||(event=civil_update(0,&s,&c,&o))<0){fprintf(stderr,"Invalid initialization/controller\n");return 1;}
    csv.file=fopen(argv[2],"w");if(!csv.file){perror("Output CSV");return 1;}
    if(remove(jsonpath)!=0&&errno!=ENOENT){perror("Remove stale summary");fclose(csv.file);return 1;}
    for(i=0;i<NCOL;++i)if(fprintf(csv.file,i?",%s":"%s",headers[i])<0)status=1;
    if(fputc('\n',csv.file)==EOF)status=1;
    min_sep=o.prediction.separation_m;status|=append(&csv,0,&s,&o,event)<0;
    while(!status&&t<duration&&!(event&BC_GROUND)){
        next=fmin(duration,(double)(step+1)*dt);if(next<=t){status=1;break;}
        while(!status&&t<next&&!(event&BC_GROUND)){
            event=bc_advance(t,next-t,&s.vehicle,&s.dynamics,&used);
            if(event<0||(used==0&&event==0)){status=1;break;}
            t=event?t+used:next;++accepted;logical=civil_update(t,&s,&c,&o);
            if(logical<0){status=1;break;}event|=logical;
            if(o.prediction.separation_m<min_sep){min_sep=o.prediction.separation_m;min_time=t;}
            max_q=fmax(max_q,o.flight.flight.quaternion_error);
            if(event&&append(&csv,t,&s,&o,event)<0)status=1;
        }
        ++step;
        if(!status&&(step%stride==0||t==duration||(event&BC_GROUND)))status=append(&csv,t,&s,&o,event)<0;
    }
    if(flush(&csv)<0)status=1;
    if(fclose(csv.file)!=0)status=1;
    if(!status&&write_summary(jsonpath,argv[1],&c,&s,&o,t,dt,min_sep,min_time,max_q,stride,accepted,csv.samples,event&BC_GROUND)<0)status=1;
    if(status){fprintf(stderr,"Simulation/output failure at %.17g s; result is incomplete\n",t);return 1;}
    printf("%s: end=%.12g s, min separation=%.9g m at %.9g s, safety activations=%d, controller calls=%lu/%lu/%lu\n",
        argv[1],t,min_sep,min_time,s.safety.activations,s.calls[0],s.calls[1],s.calls[2]);return 0;
}
