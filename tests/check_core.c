/* 解析物理算例，独立于默认案例的轨迹形状。 */
#include "trajectory.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void check(int ok, const char *msg) {
    if (!ok) { fprintf(stderr, "FAIL: %s\n", msg); exit(1); }
}
static void advance(VTParams *p, VTState *s, double duration, double dt) {
    double t = 0.0;
    while (t < duration-1e-12) {
        double h = fmin(dt, duration-t);
        vt_step(t, h, s, p, VT_CONSTANT);
        t += h;
    }
}
int main(void) {
    VTParams p; VTState s; VTObservation o; double e0, e1;
    /* 没有空气、推力、耗油时应严格退化为解析抛体运动。 */
    vt_default(&p); vt_initial(&p, &s);
    p.rho0=0.0; p.thrust_max=0.0; p.consume_fuel=0;
    s.a[0]=0.0; s.a[1]=500.0; s.a[2]=0.0;
    s.a[3]=20.0; s.a[4]=5.0; s.a[5]=3.0; s.a[7]=0.0;
    advance(&p,&s,4.0,0.02);
    check(fabs(s.a[0]-80.0)<1e-9,"free-fall forward position");
    check(fabs(s.a[1]-(500.0+20.0-0.5*p.g*16.0))<1e-9,"free-fall height");
    check(fabs(s.a[2]-12.0)<1e-9,"free-fall side position");
    check(fabs(s.a[4]-(5.0-p.g*4.0))<1e-9,"free-fall vertical velocity");
    puts("PASS: analytic free-fall");

    /* 固定指令的一阶推力响应有解析解；不比较默认轨迹截图。 */
    vt_default(&p); vt_initial(&p,&s); p.consume_fuel=0;
    p.rho0=0.0; p.g=0.0; p.thrust_max=10.0; p.throttle_mean=0.6;
    p.thrust_tau=1.5; s.a[7]=0.0;
    advance(&p,&s,3.0,0.02);
    check(fabs(s.a[7]-6.0*(1.0-exp(-2.0)))<1e-8,"analytic thrust lag");
    puts("PASS: analytic first-order thrust response");

    /* 理想升力垂直速度、不做功；能量只在数值误差范围内改变。 */
    vt_default(&p); vt_initial(&p,&s);
    p.consume_fuel=0; p.thrust_max=0.0; p.cd0=0.0; p.drag_k=0.0;
    s.a[7]=0.0;
    e0=0.5*(s.a[3]*s.a[3]+s.a[4]*s.a[4]+s.a[5]*s.a[5])+p.g*s.a[1];
    advance(&p,&s,5.0,0.005);
    e1=0.5*(s.a[3]*s.a[3]+s.a[4]*s.a[4]+s.a[5]*s.a[5])+p.g*s.a[1];
    check(fabs(e1-e0)<1e-5,"lift must not add mechanical energy");
    puts("PASS: lift orthogonality / mechanical energy");

    /* 只有0.001 kg燃料，用较大步长特意穿越耗尽事件。 */
    vt_default(&p); vt_initial(&p,&s); p.mass_dry=p.mass0-0.001;
    p.fuel_c0=0.0; p.fuel_cT=0.01;
    advance(&p,&s,1.0,0.03); vt_observe(1.0,&s,&p,VT_CONSTANT,&o);
    check(fabs(s.a[6]-p.mass_dry)<1e-12,"mass must stop at dry mass");
    check(s.a[7]==0.0 && o.thrust==0.0 && o.fuel_flow==0.0,"no thrust/fuel flow after depletion");
    puts("PASS: fuel depletion event and shutdown");

    /* 静止、竖直速度的代数基向量不能产生除零。 */
    vt_default(&p); vt_initial(&p,&s);
    for(int case_id=0;case_id<2;case_id++) {
        s.a[3]=0.0; s.a[4]=case_id ? 10.0 : 0.0; s.a[5]=0.0;
        vt_step(0.0,0.01,&s,&p,VT_VARIABLE);
        for(int j=0;j<VT_NSTATE;j++) check(isfinite(s.a[j]),"finite zero-speed / vertical state");
    }
    puts("PASS: finite low-speed and vertical fallback");
    /* 自由落体触地时间来自0.1-5t-g*t²/2=0。 */
    vt_default(&p); vt_initial(&p,&s); p.consume_fuel=0;
    p.rho0=0.0; p.thrust_max=0.0; s.a[7]=0.0;
    s.a[1]=0.1; s.a[3]=10.0; s.a[4]=-5.0; s.a[5]=0.0;
    double used=0.0;
    int hit=vt_advance(0.0,0.1,&s,&p,VT_CONSTANT,&used);
    double exact=0.2/(5.0+sqrt(25.0+0.2*p.g));
    check(hit && s.a[1]==0.0 && fabs(used-exact)<1e-11,"ground event time");
    puts("PASS: analytic ground event");
    return 0;
}
