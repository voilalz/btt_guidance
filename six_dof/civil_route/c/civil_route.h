#ifndef CIVIL_ROUTE_H
#define CIVIL_ROUTE_H
#include "boost_cruise.h"

enum { CIVIL_HIGH=0, CIVIL_CROSSING=1, CIVIL_MONITOR=2 };
enum { CIVIL_DEPARTURE=0, CIVIL_ENROUTE=1, CIVIL_APPROACH=2 };
enum { CIVIL_PHASE=16, CIVIL_WAYPOINT=32, CIVIL_SAFETY_ON=64, CIVIL_SAFETY_OFF=128, CIVIL_COMPLETE=256 };
#define CIVIL_NWAYPOINT 4

/* Flight controller input intentionally contains only own-aircraft data. */
typedef struct {
    double t_s,x_m,height_m,z_m,vx_mps,vh_mps,vz_mps,speed_mps,mass_kg;
    double roll_rad,pitch_rad,yaw_rad;
    int boost_active,cruise_ignited;
} CivilFlightView;
typedef struct { double x_m,height_m,speed_mps; } CivilSegment;
typedef struct { double entered_s,start_height_m,start_speed_mps; } CivilControllerMemory;
typedef struct { double height_m,speed_mps; } CivilReference;
typedef int (*CivilControllerFn)(const CivilFlightView*,const CivilSegment*,const CivilControllerMemory*,CivilReference*);
typedef struct { int id;const char *name;CivilControllerFn evaluate; } CivilController;
typedef struct { double x_m,height_m,z_m,vx_mps,vh_mps,vz_mps; } CivilTraffic;
typedef struct { double separation_m,horizontal_m,tcpa_s,vertical_m;int approaching; } CivilPrediction;
typedef struct { int active,activations;double entered_s,height_override_m; } CivilSafety;
typedef struct {
    BCParams base;
    const CivilController *controllers[3];
    CivilSegment waypoints[CIVIL_NWAYPOINT];
    CivilTraffic traffic0;
    double departure_time_s,lookahead_s,trigger_horizontal_m,trigger_vertical_m;
    double clearance_m,hold_s,release_horizontal_m;
    int scene,safety_enabled;
} CivilConfig;
typedef struct {
    BCState vehicle;
    BCParams dynamics;
    CivilControllerMemory memory;
    CivilReference nominal,applied;
    CivilSafety safety;
    unsigned long calls[3];
    double last_update_s;
    int phase,waypoint,route_complete;
} CivilState;
typedef struct {
    BCObservation flight;
    CivilFlightView own;
    CivilTraffic traffic;
    CivilPrediction prediction;
    CivilReference nominal,applied;
    CivilSegment segment;
    int phase,controller_id,waypoint;
} CivilObservation;

const CivilController *civil_controller(const char *name);
void civil_default(CivilConfig *c,int scene);
int civil_initial(const CivilConfig *c,CivilState *s);
int civil_traffic(double t,const CivilConfig *c,CivilTraffic *out);
int civil_cpa(const CivilFlightView *own,const CivilTraffic *traffic,double horizon,CivilPrediction *out);
int civil_supervise(double t,const CivilFlightView *own,const CivilTraffic *traffic,const CivilConfig *c,
                    const CivilReference *nominal,CivilSafety *state,CivilReference *applied,CivilPrediction *prediction);
/* Update references at an accepted time. Returns logical event bits or <0.
   Vehicle continuous state is not modified; advance it using bc_advance. */
int civil_update(double t,CivilState *s,const CivilConfig *c,CivilObservation *out);
#endif
