#ifndef BC_BOOST_CRUISE_H
#define BC_BOOST_CRUISE_H
#include "six_dof.h"

/* Original indices 0..14; 15 booster propellant, 16 cruise fuel (kg). */
#define BC_NSTATE 17
enum { BC_IGNITION=1, BC_BURNOUT=2, BC_FUEL_OUT=4, BC_GROUND=8 };
enum { BC_BOOST=0, BC_OVERLAP=1, BC_COAST=2, BC_CRUISE=3, BC_EMPTY=4 };
typedef struct {
    double a[BC_NSTATE];
    int boost_active, cruise_ignited, cruise_exhausted, airborne;
    double ignition_time;
} BCState;
typedef struct {
    SD6Params vehicle;
    double boost_thrust, burn_duration, boost_fuel0, cruise_fuel0;
    double launch_pitch, launch_height, ignition_height, ignition_speed;
    double cruise_height, cruise_speed, cruise_idle_thrust, transition_time;
    double height_gain, gamma_limit, pitch_gain, rate_gain, elevator_limit, speed_gain;
    int stabilize;
} BCParams;
typedef struct {
    SD6Observation flight;
    double boost_thrust, total_thrust, boost_flow, cruise_flow;
    double pitch_cmd, gamma_cmd;
    int stage, height_gate, speed_gate;
} BCObservation;

void bc_default(BCParams *p);
int bc_initial(BCParams *p, BCState *s);
int bc_observe(double t,const BCState *s,const BCParams *p,BCObservation *o);
int bc_rhs(double t,const BCState *s,const BCParams *p,double ds[BC_NSTATE]);
/* Stop at the earliest event; process remaining dt by calling again.
   0 = no event, positive = event bit mask, negative = invalid/numerical error. */
int bc_advance(double t,double dt,BCState *s,const BCParams *p,double *used_dt);
#endif
