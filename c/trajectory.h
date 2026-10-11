#ifndef VARIABLE_THRUST_TRAJECTORY_H
#define VARIABLE_THRUST_TRAJECTORY_H

/* 全部采用SI单位；角度为rad。坐标顺序x前、y上、z右。 */
#define VT_NSTATE 8
typedef enum { VT_CONSTANT=0, VT_VARIABLE=1 } VTMode;
typedef struct { double a[VT_NSTATE]; } VTState;
typedef struct {
    double g, rho0, scale_height, area;
    double cl0, cl_alpha, cl_min, cl_max, cd0, drag_k;
    double mass0, mass_dry, thrust_max, thrust_tau;
    double fuel_c0, fuel_cT; int consume_fuel;
    double speed0, height0, alpha_trim;
    double alpha_amp, alpha_period, bank_amp, bank_period;
    double throttle_mean, throttle_amp1, throttle_period1;
    double throttle_amp2, throttle_period2;
} VTParams;
typedef struct {
    double speed, theta, psi_v, alpha, bank, rho, lift, drag;
    double throttle, thrust_cmd, thrust, fuel_flow;
} VTObservation;

void vt_default(VTParams *p);
void vt_initial(const VTParams *p, VTState *s);
void vt_observe(double t,const VTState *s,const VTParams *p,VTMode mode,VTObservation *o);
void vt_rhs(double t,const VTState *s,const VTParams *p,VTMode mode,VTState *ds);
void vt_step(double t,double dt,VTState *s,const VTParams *p,VTMode mode);
/* 返回1代表触地；used_dt是本次实际积分时长。 */
int vt_advance(double t,double dt,VTState *s,const VTParams *p,VTMode mode,double *used_dt);
#endif
