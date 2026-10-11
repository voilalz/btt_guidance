#include "civil_route.h"
#include <math.h>
#include <string.h>

static double clip(double a,double lo,double hi){return fmax(lo,fmin(hi,a));}
static int direct(const CivilFlightView *v,const CivilSegment *g,const CivilControllerMemory *m,CivilReference *r){
    (void)v;(void)m;if(!g||!r)return -1;
    r->height_m=g->height_m;r->speed_mps=g->speed_mps;return 0;
}
static int launch(const CivilFlightView *v,const CivilSegment *g,const CivilControllerMemory *m,CivilReference *r){
    (void)v;(void)g;(void)m;if(!r)return -1;r->height_m=40;r->speed_mps=25;return 0;
}
static int slew(const CivilFlightView *v,const CivilSegment *g,const CivilControllerMemory *m,CivilReference *r){
    double elapsed;
    if(!v||!g||!m||!r||!isfinite(v->t_s)||!isfinite(m->entered_s)||v->t_s<m->entered_s)return -1;
    elapsed=v->t_s-m->entered_s;
    r->height_m=m->start_height_m+clip(g->height_m-m->start_height_m,-elapsed,elapsed);
    r->speed_mps=m->start_speed_mps+clip(g->speed_mps-m->start_speed_mps,-.5*elapsed,.5*elapsed);
    return 0;
}
const CivilController *civil_controller(const char *name){
    static const CivilController controllers[]={{0,"launch",launch},{1,"direct",direct},{2,"slew",slew}};
    unsigned i;if(!name)return NULL;
    for(i=0;i<sizeof(controllers)/sizeof(controllers[0]);++i)
        if(strcmp(name,controllers[i].name)==0)return &controllers[i];
    return NULL;
}
