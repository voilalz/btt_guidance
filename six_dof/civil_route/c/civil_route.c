#include "civil_route.h"
#include <math.h>
#include <string.h>

static int valid_reference(const CivilReference *r){
    return r&&isfinite(r->height_m)&&isfinite(r->speed_mps)&&r->height_m>=1&&r->height_m<=300&&
        r->speed_mps>=15&&r->speed_mps<=40;
}
static int valid_traffic(const CivilTraffic *a){
    return a&&isfinite(a->x_m)&&isfinite(a->height_m)&&isfinite(a->z_m)&&isfinite(a->vx_mps)&&
        isfinite(a->vh_mps)&&isfinite(a->vz_mps);
}
static int valid_config(const CivilConfig *c){
    int i;
    if(!c||!valid_traffic(&c->traffic0)||c->scene<0||c->scene>2)return 0;
    {
        const double a[]={c->departure_time_s,c->lookahead_s,c->trigger_horizontal_m,c->trigger_vertical_m,
                          c->clearance_m,c->hold_s,c->release_horizontal_m};
        for(i=0;i<7;++i)if(!isfinite(a[i])||a[i]<=0)return 0;
    }
    if(c->release_horizontal_m<=c->trigger_horizontal_m)return 0;
    for(i=0;i<3;++i)if(!c->controllers[i]||!c->controllers[i]->name||!c->controllers[i]->evaluate)return 0;
    for(i=0;i<CIVIL_NWAYPOINT;++i){
        CivilReference r={c->waypoints[i].height_m,c->waypoints[i].speed_mps};
        if(!valid_reference(&r)||!isfinite(c->waypoints[i].x_m)||c->waypoints[i].x_m<=0||
           (i&&c->waypoints[i].x_m<=c->waypoints[i-1].x_m))return 0;
    }
    return 1;
}
void civil_default(CivilConfig *c,int scene){
    const CivilSegment route[CIVIL_NWAYPOINT]={{800,40,25},{1800,60,25},{2200,40,25},{2800,30,20}};
    memset(c,0,sizeof(*c));bc_default(&c->base);memcpy(c->waypoints,route,sizeof(route));
    c->controllers[0]=civil_controller("launch");c->controllers[1]=civil_controller("direct");c->controllers[2]=civil_controller("slew");
    c->departure_time_s=8;c->lookahead_s=30;c->trigger_horizontal_m=120;c->trigger_vertical_m=100;
    c->clearance_m=120;c->hold_s=40;c->release_horizontal_m=300;c->scene=scene;c->safety_enabled=scene==CIVIL_CROSSING;
    c->traffic0=scene==CIVIL_HIGH?(CivilTraffic){6000,10000,0,-50,0,0}:(CivilTraffic){1500,40,-1000,0,0,20};
}
int civil_initial(const CivilConfig *c,CivilState *s){
    if(!s||!valid_config(c))return -1;
    memset(s,0,sizeof(*s));s->dynamics=c->base;
    if(bc_initial(&s->dynamics,&s->vehicle)<0)return -1;
    s->phase=-1;s->last_update_s=-1;
    s->nominal=(CivilReference){c->base.cruise_height,c->base.cruise_speed};s->applied=s->nominal;
    return 0;
}
int civil_traffic(double t,const CivilConfig *c,CivilTraffic *out){
    if(!c||!out||!isfinite(t)||t<0||!valid_traffic(&c->traffic0))return -1;
    *out=c->traffic0;out->x_m+=t*out->vx_mps;out->height_m+=t*out->vh_mps;out->z_m+=t*out->vz_mps;
    return valid_traffic(out)?0:-1;
}
int civil_cpa(const CivilFlightView *own,const CivilTraffic *traffic,double horizon,CivilPrediction *out){
    double dx,dz,dh,dvx,dvz,v2,dot,t;
    if(!own||!out||!valid_traffic(traffic)||!isfinite(horizon)||horizon<0||
       !isfinite(own->x_m)||!isfinite(own->height_m)||!isfinite(own->z_m)||
       !isfinite(own->vx_mps)||!isfinite(own->vz_mps))return -1;
    dx=traffic->x_m-own->x_m;dz=traffic->z_m-own->z_m;dh=traffic->height_m-own->height_m;
    dvx=traffic->vx_mps-own->vx_mps;dvz=traffic->vz_mps-own->vz_mps;
    v2=dvx*dvx+dvz*dvz;dot=dx*dvx+dz*dvz;
    if(!isfinite(v2)||!isfinite(dot))return -1;
    t=v2>1e-16?fmax(0,fmin(horizon,-dot/v2)):0;
    out->tcpa_s=t;out->horizontal_m=hypot(dx+t*dvx,dz+t*dvz);
    out->separation_m=hypot(hypot(dx,dz),dh);out->vertical_m=fabs(dh);out->approaching=dot<0;
    return isfinite(out->horizontal_m)&&isfinite(out->separation_m)?0:-1;
}
int civil_supervise(double t,const CivilFlightView *own,const CivilTraffic *traffic,const CivilConfig *c,
                    const CivilReference *nominal,CivilSafety *state,CivilReference *applied,CivilPrediction *prediction){
    int event=0,risk;double horizontal,relative_dot;
    if(!state||!applied||!isfinite(t)||t<0||!valid_config(c)||!valid_reference(nominal)||
       civil_cpa(own,traffic,c->lookahead_s,prediction)<0||
       !isfinite(state->entered_s)||!isfinite(state->height_override_m))return -1;
    *applied=*nominal;horizontal=hypot(traffic->x_m-own->x_m,traffic->z_m-own->z_m);
    relative_dot=(traffic->x_m-own->x_m)*(traffic->vx_mps-own->vx_mps)+
        (traffic->z_m-own->z_m)*(traffic->vz_mps-own->vz_mps);
    risk=prediction->approaching&&prediction->horizontal_m<c->trigger_horizontal_m&&
        prediction->vertical_m<c->trigger_vertical_m;
    if(!c->safety_enabled){if(state->active)event=CIVIL_SAFETY_OFF;state->active=0;return event;}
    if(state->active&&t-state->entered_s>=c->hold_s&&horizontal>c->release_horizontal_m&&relative_dot>0){
        state->active=0;event=CIVIL_SAFETY_OFF;
    }
    if(!state->active&&risk&&own->cruise_ignited&&!own->boost_active){
        state->active=1;state->entered_s=t;state->height_override_m=traffic->height_m+c->clearance_m;
        ++state->activations;event|=CIVIL_SAFETY_ON;
    }
    if(state->active)applied->height_m=fmax(nominal->height_m,state->height_override_m);
    return valid_reference(applied)?event:-1;
}
static void flight_view(double t,const CivilState *s,const BCObservation *o,CivilFlightView *v){
    v->t_s=t;v->x_m=s->vehicle.a[0];v->height_m=-s->vehicle.a[2];v->z_m=s->vehicle.a[1];
    v->vx_mps=o->flight.vn[0];v->vh_mps=-o->flight.vn[2];v->vz_mps=o->flight.vn[1];
    v->speed_mps=o->flight.speed;v->mass_kg=s->vehicle.a[13];
    v->roll_rad=o->flight.roll;v->pitch_rad=o->flight.pitch;v->yaw_rad=o->flight.yaw;
    v->boost_active=s->vehicle.boost_active;v->cruise_ignited=s->vehicle.cruise_ignited;
}
int civil_update(double t,CivilState *s,const CivilConfig *c,CivilObservation *out){
    CivilObservation o;SD6Params trim;SD6State dummy;CivilState next;int phase,wp,event=0,e;
    if(!s||!out||!valid_config(c)||!isfinite(t)||t<0||t<s->last_update_s||s->waypoint<0||s->waypoint>=CIVIL_NWAYPOINT)return -1;
    next=*s;memset(&o,0,sizeof(o));
    if(bc_observe(t,&s->vehicle,&s->dynamics,&o.flight)<0)return -1;
    flight_view(t,s,&o.flight,&o.own);wp=s->waypoint;
    while(wp<CIVIL_NWAYPOINT-1&&o.own.x_m>=c->waypoints[wp].x_m)++wp;
    phase=(t>=c->departure_time_s&&s->vehicle.cruise_ignited&&!s->vehicle.boost_active)?
          (wp==CIVIL_NWAYPOINT-1?CIVIL_APPROACH:CIVIL_ENROUTE):CIVIL_DEPARTURE;
    if(phase!=s->phase)event|=CIVIL_PHASE;
    if(wp!=s->waypoint)event|=CIVIL_WAYPOINT;
    if(phase!=s->phase||wp!=s->waypoint){
        next.memory=(CivilControllerMemory){t,s->applied.height_m,s->applied.speed_mps};
    }
    next.phase=phase;next.waypoint=wp;o.phase=phase;o.waypoint=wp;o.segment=c->waypoints[wp];
    o.controller_id=c->controllers[phase]->id;
    if(c->controllers[phase]->evaluate(&o.own,&o.segment,&next.memory,&o.nominal)<0||!valid_reference(&o.nominal))return -1;
    ++next.calls[phase];
    if(civil_traffic(t,c,&o.traffic)<0)return -1;
    e=civil_supervise(t,&o.own,&o.traffic,c,&o.nominal,&next.safety,&o.applied,&o.prediction);
    if(e<0)return -1;
    event|=e;next.nominal=o.nominal;next.applied=o.applied;
    next.dynamics.cruise_height=o.applied.height_m;next.dynamics.cruise_speed=o.applied.speed_mps;
    /* Solve only a reference trim; the dummy state never replaces flight state. */
    trim=next.dynamics.vehicle;trim.mass0=s->vehicle.a[13];trim.height0=o.applied.height_m;trim.speed0=o.applied.speed_mps;
    if(sd6_initial(&trim,&dummy)<0)return -1;
    next.dynamics.vehicle.alpha_trim=trim.alpha_trim;next.dynamics.vehicle.elevator_trim=trim.elevator_trim;
    next.dynamics.vehicle.throttle_trim=trim.throttle_trim;
    if(bc_observe(t,&s->vehicle,&next.dynamics,&o.flight)<0)return -1;
    if(!next.route_complete&&o.own.x_m>=c->waypoints[CIVIL_NWAYPOINT-1].x_m){next.route_complete=1;event|=CIVIL_COMPLETE;}
    next.last_update_s=t;*s=next;*out=o;return event;
}
