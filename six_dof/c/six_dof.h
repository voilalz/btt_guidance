#ifndef SD6_SIX_DOF_H
#define SD6_SIX_DOF_H

/* SI, radians; state: N,E,D,u,v,w,a,b,c,d,p,q,r,m,T. */
#define SD6_NSTATE 15
typedef enum { SD6_CONSTANT = 0, SD6_VARIABLE = 1 } SD6Mode;
typedef struct { double a[SD6_NSTATE]; } SD6State;
typedef struct {
    double g, rho0, scale_height, area, span, chord, jx, jy, jz;
    double cl0, cl_alpha, cl_min, cl_max, cd0, drag_k, cy_beta, cy_r;
    double roll_beta, roll_p, roll_r, roll_a;
    double pitch0, pitch_alpha, pitch_q, pitch_e;
    double yaw_beta, yaw_p, yaw_r, yaw_rudder;
    double mass0, mass_dry, thrust_max, thrust_tau, fuel_c0, fuel_cT;
    double speed0, height0, alpha_trim, elevator_trim, throttle_trim;
    double throttle_override; /* <0: demo schedule; >=0: constant input */
    int consume_fuel, aero_enabled, disturbances;
} SD6Params;
typedef struct {
    double vn[3], force[3], moment[3];
    double speed, alpha, beta, rho, lift, drag, side;
    double roll, pitch, yaw, flight_path, psi_v, quaternion_error;
    double throttle, thrust_cmd, fuel_flow, da, de, dr;
} SD6Observation;

void sd6_default(SD6Params *p);
/* Solve full force/moment trim and construct the initial state. */
int sd6_initial(SD6Params *p, SD6State *s);
/* Body -> NED, scalar-first Hamilton quaternion; normalized copy. */
int sd6_rotation(const double quaternion[4], double r[3][3]);
int sd6_observe(double t, const SD6State *s, const SD6Params *p,
                SD6Mode mode, SD6Observation *o);
int sd6_rhs(double t, const SD6State *s, const SD6Params *p,
            SD6Mode mode, SD6State *ds);
/* One RK4 substep with propulsion switch fixed. Does not locate events. */
int sd6_step(double t, double dt, SD6State *s, const SD6Params *p,
             SD6Mode mode, int engine_on);
/* 0=normal, 1=ground, negative=invalid/numerical failure. Locates dry/ground events. */
int sd6_advance(double t, double dt, SD6State *s, const SD6Params *p,
                SD6Mode mode, double *used_dt);

#endif
