#include "six_dof.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static void check(int ok, const char *name) {
    if (!ok) { fprintf(stderr, "FAIL: %s\n", name); ++failures; }
}
static double dot(const double *a, const double *b, int n) {
    double v=0; int i; for(i=0;i<n;++i) v+=a[i]*b[i]; return v;
}
static void base(SD6Params *p, SD6State *s) {
    sd6_default(p); memset(s,0,sizeof(*s)); s->a[6]=1;
    s->a[2]=-200; s->a[13]=12; p->aero_enabled=0;
    p->consume_fuel=0; p->thrust_max=0; p->disturbances=0;
}
static int integrate(SD6State *s, SD6Params *p, double end, double dt) {
    int i, n=(int)lround(end/dt);
    for(i=0;i<n;++i) if(sd6_step(i*dt,dt,s,p,SD6_CONSTANT,1)<0) return 0;
    return 1;
}
static void rotation_tests(void) {
    double q[4]={1,0,0,0}, R[3][3], det; int i,j,k;
    check(sd6_rotation(q,R)==0,"identity rotation available");
    check(fabs(R[0][0]-1)+fabs(R[1][1]-1)+fabs(R[2][2]-1)<1e-14,"identity matrix");
    q[0]=sqrt(.5);q[3]=sqrt(.5);
    check(sd6_rotation(q,R)==0 && fabs(R[1][0]-1)<1e-14 && fabs(R[0][1]+1)<1e-14,"positive yaw turns forward to right");
    q[0]=cos(.3);q[1]=sin(.3);q[2]=q[3]=0; sd6_rotation(q,R);
    check(-R[1][2]>0,"positive bank tilts upward lift toward right");
    q[0]=.2;q[1]=-.3;q[2]=.7;q[3]=.9; sd6_rotation(q,R);
    for(i=0;i<3;++i) for(j=0;j<3;++j) {
        double a=0;for(k=0;k<3;++k)a+=R[k][i]*R[k][j];
        check(fabs(a-(i==j))<1e-14,"orthogonal rotation");
    }
    det=R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])-R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])+R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]);
    check(fabs(det-1)<1e-14,"proper rotation determinant");
    memset(q,0,sizeof(q));check(sd6_rotation(q,R)<0,"reject zero quaternion");
    q[0]=NAN;check(sd6_rotation(q,R)<0,"reject nonfinite quaternion");
    if(!failures) puts("PASS: rotation signs, orthogonality and invalid quaternions");
}
static void trim_and_aero_tests(void) {
    SD6Params p;SD6State s,d;SD6Observation o;double power;int i,ok;
    sd6_default(&p);ok=sd6_initial(&p,&s)==0;check(ok,"trim exists"); if(!ok)return;
    check(sd6_rhs(0,&s,&p,SD6_CONSTANT,&d)==0,"trim rhs");
    for(i=3;i<6;++i)check(fabs(d.a[i])<1e-8,"full force trim derivative");
    for(i=10;i<13;++i)check(fabs(d.a[i])<1e-8,"full moment trim derivative");
    check(fabs(p.alpha_trim-2.438596874881059*acos(-1)/180)<1e-12,"independent trim root");
    check(fabs(s.a[14]-13.232733930207198)<1e-9,"trim thrust includes inclination");
    p.consume_fuel=0;p.disturbances=0;
    check(integrate(&s,&p,10,.01),"frozen-mass trim integration");
    check(fabs(s.a[0]-250)<1e-8 && fabs(s.a[1])<1e-8 && fabs(s.a[2]+200)<1e-8,"trim horizontal flight");
    s.a[3]=23;s.a[4]=2;s.a[5]=3;
    check(sd6_observe(3,&s,&p,SD6_CONSTANT,&o)==0,"aero observation");
    power=dot(o.force,s.a+3,3);
    check(fabs(power+o.drag*o.speed)<1e-9,"lift and side power zero, drag dissipative");
    s.a[3]=s.a[4]=s.a[5]=0;sd6_observe(3,&s,&p,SD6_CONSTANT,&o);
    check(dot(o.force,o.force,3)==0 && dot(o.moment,o.moment,3)==0 && o.alpha==0 && o.beta==0,"zero speed aerodynamics finite and zero");
    p.mass0=1000;check(sd6_initial(&p,&s)<0,"unavailable trim rejected");
    if(!failures)puts("PASS: full trim, horizontal flight, aero power and low-speed limit");
}
static void freefall_tests(void) {
    int k,i;double pitches[]={0,.7,-1.1};
    for(k=0;k<3;++k){
        SD6Params p;SD6State s;SD6Observation o;double R[3][3],v0[]={11,-2,3};
        base(&p,&s);s.a[6]=cos(pitches[k]/2);s.a[8]=sin(pitches[k]/2);sd6_rotation(s.a+6,R);
        for(i=0;i<3;++i)s.a[3+i]=R[0][i]*v0[0]+R[1][i]*v0[1]+R[2][i]*v0[2];
        check(integrate(&s,&p,2,.01),"freefall integration");sd6_observe(2,&s,&p,SD6_CONSTANT,&o);
        check(fabs(s.a[0]-22)<1e-9 && fabs(s.a[1]+4)<1e-9 && fabs(s.a[2]-(-200+6+2*p.g))<1e-9,"analytic inertial freefall position");
        check(fabs(o.vn[0]-11)<1e-10 && fabs(o.vn[1]+2)<1e-10 && fabs(o.vn[2]-(3+2*p.g))<1e-10,"analytic gravity under rotated body frame");
    }if(!failures)puts("PASS: freefall in multiple body attitudes");
}
static void sphere_tests(void) {
    SD6Params p;SD6State s;SD6Observation o;double omega[3]={.2,-.3,.4},n=sqrt(.29);int i;
    base(&p,&s);p.g=0;p.jx=p.jy=p.jz=2;s.a[3]=15;
    for(i=0;i<3;++i)s.a[10+i]=omega[i];
    check(integrate(&s,&p,5,.01),"sphere integration");sd6_observe(5,&s,&p,SD6_CONSTANT,&o);
    check(fabs(s.a[6]-cos(n*2.5))<1e-11,"analytic scalar quaternion");
    for(i=0;i<3;++i)check(fabs(s.a[7+i]-omega[i]/n*sin(n*2.5))<1e-11,"analytic vector quaternion");
    check(fabs(o.vn[0]-15)+fabs(o.vn[1])+fabs(o.vn[2])<1e-8,"inertial velocity conserved with rotating body");
    check(fabs(s.a[0]-75)+fabs(s.a[1])+fabs(s.a[2]+200)<1e-8,"inertial position in rotating coordinates");
    if(!failures)puts("PASS: sphere quaternion and rotating-coordinate translation");
}
static double asymmetric_error(double dt) {
    SD6Params p;SD6State s;double R[3][3],w[3]={.7,1.1,-.9},j[3],energy0,energy,h[3],h0[3];int i,k;
    base(&p,&s);p.g=0;j[0]=p.jx;j[1]=p.jy;j[2]=p.jz;
    energy0=0;for(i=0;i<3;++i){s.a[10+i]=w[i];h0[i]=j[i]*w[i];energy0+=.5*j[i]*w[i]*w[i];}
    check(integrate(&s,&p,20,dt),"asymmetric rigidbody integration");sd6_rotation(s.a+6,R);
    energy=0;for(i=0;i<3;++i)energy+=.5*j[i]*s.a[10+i]*s.a[10+i];
    for(i=0;i<3;++i){h[i]=0;for(k=0;k<3;++k)h[i]+=R[i][k]*j[k]*s.a[10+k];}
    return fabs(energy-energy0)+fabs(h[0]-h0[0])+fabs(h[1]-h0[1])+fabs(h[2]-h0[2]);
}
static void engine_tests(void) {
    SD6Params p;SD6State s;double e1,e2;
    base(&p,&s);p.g=0;p.thrust_max=60;p.throttle_override=.6;s.a[14]=2;
    check(integrate(&s,&p,5,.01),"engine step integration");
    check(fabs(s.a[14]-(36+(2-36)*exp(-5)))<1e-9,"analytic first-order thrust lag");
    e1=asymmetric_error(.02);e2=asymmetric_error(.01);
    check(e1<1e-6 && e2<e1*.15,"rigidbody energy/angular momentum and fourth-order convergence");
    if(!failures)printf("PASS: engine lag; asymmetric conservation error %.3g -> %.3g\n",e1,e2);
}
static void invalid_parameter_tests(void) {
    SD6Params p;SD6State s,d;SD6Observation o;double used;
    sd6_default(&p);check(sd6_initial(&p,&s)==0,"valid setup for NaN parameter test");
    p.cl_alpha=NAN;
    check(sd6_rhs(0,&s,&p,SD6_CONSTANT,&d)<0,"NaN lift slope rejected before clipping");
    check(sd6_observe(0,&s,&p,SD6_CONSTANT,&o)<0,"NaN coefficient observation rejected");
    check(sd6_advance(0,.01,&s,&p,SD6_CONSTANT,&used)<0,"NaN coefficient cannot advance");
    sd6_default(&p);p.pitch0=NAN;
    check(sd6_initial(&p,&s)<0,"NaN pitch coefficient cannot produce successful trim");
    sd6_default(&p);sd6_initial(&p,&s);p.throttle_override=NAN;
    check(sd6_rhs(0,&s,&p,SD6_CONSTANT,&d)<0,"NaN input override rejected");
    sd6_default(&p);p.cl_max=NAN;
    check(sd6_initial(&p,&s)<0,"NaN lift clipping bound rejected");
    sd6_default(&p);p.pitch0=DBL_MAX;p.pitch_e=1e-11;
    check(sd6_initial(&p,&s)<0,"overflowed trim output cannot report success");
    sd6_default(&p);sd6_initial(&p,&s);p.elevator_trim=INFINITY;
    check(sd6_rhs(0,&s,&p,SD6_CONSTANT,&d)<0,"nonfinite trim input rejected");
    if(!failures)puts("PASS: nonfinite coefficients, clipping bounds and trim/input parameters rejected");
}
int main(void){rotation_tests();trim_and_aero_tests();freefall_tests();sphere_tests();engine_tests();invalid_parameter_tests();return failures?1:0;}
