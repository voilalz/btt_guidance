function summary = run_boost_cruise_demo(outputDir,duration,dt,outputStride)
% Static launch -> solid booster -> AND-gate cruise ignition, full six DOF.
% Independent MATLAB implementation; SI units and CSV radians, no toolbox.
if nargin<1 || isempty(outputDir)
    outputDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results_matlab');
end
if nargin<2, duration=120; end
if nargin<3, dt=0.005; end
if nargin<4, outputStride=10; end
assert(isscalar(duration)&&isfinite(duration)&&duration>0&&duration<=3600,'Invalid duration');
assert(isscalar(dt)&&isfinite(dt)&&dt>0&&ceil(duration/dt)<=2000000,'Invalid dt/step count');
assert(isscalar(outputStride)&&isfinite(outputStride)&&outputStride>=1&&outputStride==floor(outputStride),'Invalid output stride');
if ~exist(outputDir,'dir'),mkdir(outputDir);end
P=defaults();[P,s]=initial(P);t=0;regularStep=0;event=0;
rows=zeros(floor(ceil(duration/dt)/outputStride)+10,58);count=1;rows(1,:)=sample(0,s,P,0);
while t<duration && bitand(event,8)==0
    next=min(duration,(regularStep+1)*dt);assert(next>t,'Floating-point time cannot advance');
    while t<next && bitand(event,8)==0
        [s,used,event]=advance(t,next-t,s,P);
        assert(used>0 || event~=0,'Event integrator did not advance');
        if event~=0,t=t+used;else,t=next;end
        if event~=0,[rows,count]=append(rows,count,t,s,P,event);end
    end
    regularStep=regularStep+1;
    if mod(regularStep,outputStride)==0 || t==duration || bitand(event,8)~=0
        [rows,count]=append(rows,count,t,s,P,event);
    end
end
rows=rows(1:count,:);
headers={'t_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','N_m','E_m','D_m', ...
    'u_mps','v_mps','w_mps','quat_a','quat_b','quat_c','quat_d','p_radps','q_radps','r_radps', ...
    'roll_rad','pitch_rad','yaw_rad','alpha_rad','beta_rad','flight_path_rad','psi_v_rad', ...
    'mass_kg','thrust_N','thrust_cmd_N','throttle','delta_a_rad','delta_e_rad','delta_r_rad', ...
    'lift_N','drag_N','side_N','Mx_Nm','My_Nm','Mz_Nm','quat_norm_error','speed_mps', ...
    'boost_thrust_N','total_thrust_N','boost_fuel_kg','cruise_fuel_kg','boost_flow_kgps','cruise_flow_kgps', ...
    'boost_active','cruise_ignited','cruise_exhausted','stage','ignition_time_s','pitch_cmd_rad', ...
    'gamma_cmd_rad','height_gate','speed_gate','event_mask'};
write_csv(fullfile(outputDir,'boost_cruise.csv'),headers,rows);
summary.parameters=P;summary.dt_s=dt;summary.output_stride=outputStride;summary.samples=count;
summary.end_time_s=t;summary.ignition_time_s=s.ignition_time;
summary.final_position_m=[s.a(1),-s.a(3),s.a(2)];summary.final_speed_mps=norm(s.a(4:6));
if bitand(event,8),summary.termination='ground';else,summary.termination='duration';end
save(fullfile(outputDir,'summary.mat'),'summary');plot_saved_boost_cruise(outputDir);
fprintf('boost/cruise: %d samples, end=%.12g s, ignition=%.12g s\n',count,t,s.ignition_time);
end

function P=defaults()
v.g=9.80665;v.rho0=1.225;v.scale_height=8500;v.area=0.8;v.span=2;v.chord=0.4;v.J=[1;1.6;2.2];
v.cl0=0.2;v.cl_alpha=4.5;v.cl_min=-0.6;v.cl_max=1.2;v.cd0=0.035;v.drag_k=0.06;v.cy_beta=-0.5;
v.roll_beta=-0.08;v.roll_p=-0.45;v.roll_r=0.10;v.pitch0=0.03;v.pitch_alpha=-0.80;v.pitch_q=-5;v.pitch_e=-1;
v.yaw_beta=0.08;v.yaw_p=-0.04;v.yaw_r=-0.20;v.mass_dry=11.7;
v.thrust_max=60;v.thrust_tau=1;v.fuel_c0=5e-5;v.fuel_cT=1.5e-5;v.aero_enabled=true;
P.vehicle=v;P.boost_thrust=260;P.burn_duration=4;P.boost_fuel0=0.5;P.cruise_fuel0=0.3;
P.launch_pitch=35*pi/180;P.launch_height=0;P.ignition_height=40;P.ignition_speed=25;
P.cruise_height=40;P.cruise_speed=25;P.cruise_idle_thrust=3;P.transition_time=3;
P.height_gain=0.02;P.gamma_limit=8*pi/180;P.pitch_gain=0.8;P.rate_gain=0.25;
P.elevator_limit=20*pi/180;P.speed_gain=5;P.stabilize=true;
end

function [P,s]=initial(P)
values=struct2cell(P.vehicle);assert(all(cellfun(@(x)all(isfinite(x(:))),values)),'Nonfinite vehicle parameter');
fields=fieldnames(P);
for k=1:numel(fields)
    if ~strcmp(fields{k},'vehicle'),assert(all(isfinite(P.(fields{k}))),'Nonfinite scenario parameter');end
end
assert(P.boost_thrust>0 && P.burn_duration>0 && P.boost_fuel0>0 && P.cruise_fuel0>0,'Invalid booster/fuel');
assert(P.launch_height>=0 && P.launch_pitch>0 && P.launch_pitch<pi/2,'Invalid launch');
assert(P.ignition_height>=0 && P.ignition_speed>0 && P.cruise_height>0 && P.cruise_speed>0,'Invalid gates/references');
assert(P.transition_time>0 && P.gamma_limit>0 && P.gamma_limit<pi/2 && P.elevator_limit>0,'Invalid controller limits');
assert(P.height_gain>=0 && P.pitch_gain>=0 && P.rate_gain>=0 && P.speed_gain>=0,'Invalid controller gains');
assert(isfinite(P.boost_fuel0/P.burn_duration),'Invalid booster flow');
assert(P.cruise_idle_thrust>=0 && P.cruise_idle_thrust<=P.vehicle.thrust_max,'Invalid cruise idle thrust');
v=P.vehicle;lo=-15*pi/180;hi=15*pi/180;flo=trim_residual(lo,P);fhi=trim_residual(hi,P);
assert(isfinite(flo)&&isfinite(fhi)&&flo*fhi<=0,'No reference cruise trim');
for k=1:80
    mid=(lo+hi)/2;fm=trim_residual(mid,P);
    if abs(fm)<1e-13,break;end
    if (fm>0)==(flo>0),lo=mid;flo=fm;else,hi=mid;end
end
v.alpha_trim=mid;v.elevator_trim=-(v.pitch0+v.pitch_alpha*mid)/v.pitch_e;
rho=v.rho0*exp(-P.cruise_height/v.scale_height);cl=clip(v.cl0+v.cl_alpha*mid,v.cl_min,v.cl_max);
T=0.5*rho*P.cruise_speed^2*v.area*(v.cd0+v.drag_k*cl^2)/cos(mid);
assert(isfinite(T)&&T>=0&&T<=v.thrust_max&&isfinite(v.elevator_trim),'Invalid reference trim');
v.throttle_trim=T/v.thrust_max;P.vehicle=v;
s.a=zeros(17,1);s.a(3)=-P.launch_height;s.a(7)=cos(P.launch_pitch/2);s.a(9)=sin(P.launch_pitch/2);
s.a(16)=P.boost_fuel0;s.a(17)=P.cruise_fuel0;s.a(14)=v.mass_dry+s.a(16)+s.a(17);
s.boost_active=true;s.cruise_ignited=false;s.cruise_exhausted=false;s.airborne=P.launch_height>0;s.ignition_time=-1;
assert(P.launch_height>0 || P.boost_thrust*sin(P.launch_pitch)/s.a(14)>v.g,'Insufficient upward release acceleration');
end

function f=trim_residual(alpha,P)
v=P.vehicle;qs=0.5*v.rho0*exp(-P.cruise_height/v.scale_height)*P.cruise_speed^2*v.area;
cl=clip(v.cl0+v.cl_alpha*alpha,v.cl_min,v.cl_max);
f=qs*cl+qs*(v.cd0+v.drag_k*cl^2)*tan(alpha)-(v.mass_dry+P.cruise_fuel0)*v.g;
end

function R=rotation(q)
n=norm(q);assert(isfinite(n)&&n>=1e-12,'Invalid quaternion');q=q/n;a=q(1);b=q(2);c=q(3);d=q(4);
R=[a*a+b*b-c*c-d*d,2*(b*c-a*d),2*(b*d+a*c); ...
   2*(b*c+a*d),a*a-b*b+c*c-d*d,2*(c*d-a*b); ...
   2*(b*d-a*c),2*(c*d+a*b),a*a-b*b-c*c+d*d];
end

function o=observe(t,s,P)
a=s.a;v=P.vehicle;assert(all(isfinite(a))&&a(14)>0,'Invalid state');R=rotation(a(7:10));Q=a(7:10)/norm(a(7:10));
q0=Q(1);q1=Q(2);q2=Q(3);q3=Q(4);on=s.cruise_ignited && ~s.cruise_exhausted;
o.roll=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));o.pitch=asin(clip(2*(q0*q2-q3*q1),-1,1));
o.yaw=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));o.vn=R*a(4:6);o.speed=norm(a(4:6));
o.flight_path=atan2(-o.vn(3),hypot(o.vn(1),o.vn(2)));o.psi_v=atan2(-o.vn(2),o.vn(1));
o.alpha=0;o.beta=0;o.lift=0;o.drag=0;o.side=0;o.force=zeros(3,1);o.moment=zeros(3,1);
if o.speed>=1e-8,o.alpha=atan2(a(6),a(4));o.beta=atan2(a(5),hypot(a(4),a(6)));end
o.boost_thrust=double(s.boost_active)*P.boost_thrust;o.boost_flow=double(s.boost_active)*P.boost_fuel0/P.burn_duration;
o.cruise_flow=double(on)*(v.fuel_c0+v.fuel_cT*a(15));o.total_thrust=o.boost_thrust+double(on)*a(15);
o.height_gate=-a(3)>=P.ignition_height;o.speed_gate=o.speed>=P.ignition_speed;
if s.boost_active
    if on,o.stage=1;else,o.stage=0;end
elseif on,o.stage=3;elseif s.cruise_ignited,o.stage=4;else,o.stage=2;end
rho=v.rho0*exp(-max(-a(3),0)/v.scale_height);qbar=0.5*rho*o.speed^2;
if o.speed>=1e-8 && v.aero_enabled
    cl=clip(v.cl0+v.cl_alpha*o.alpha,v.cl_min,v.cl_max);cd=v.cd0+v.drag_k*cl^2;
    o.lift=qbar*v.area*cl;o.drag=qbar*v.area*cd;o.side=qbar*v.area*v.cy_beta*o.beta;
    ca=cos(o.alpha);sa=sin(o.alpha);cb=cos(o.beta);sb=sin(o.beta);
    o.force=-o.drag*[ca*cb;sb;sa*cb]+o.side*[-ca*sb;cb;-sa*sb]-o.lift*[-sa;0;ca];
end
o.pitch_cmd=P.launch_pitch;o.gamma_cmd=0;o.de=v.elevator_trim;o.cmd=double(on)*v.thrust_max*v.throttle_trim;
if P.stabilize
    if s.cruise_ignited
        o.gamma_cmd=clip(P.height_gain*(P.cruise_height+a(3)),-P.gamma_limit,P.gamma_limit);
        blend=clip((t-s.ignition_time)/P.transition_time,0,1);blend=blend^2*(3-2*blend);
        o.pitch_cmd=(1-blend)*P.launch_pitch+blend*(v.alpha_trim+o.gamma_cmd);
    end
    eff=-(v.pitch0+v.pitch_alpha*o.alpha)/v.pitch_e;
    o.de=clip(eff+P.pitch_gain*(o.pitch-o.pitch_cmd)+P.rate_gain*a(12),-P.elevator_limit,P.elevator_limit);
    if on,o.cmd=(o.drag+(v.mass_dry+a(16)+a(17))*v.g*sin(o.gamma_cmd))/max(cos(o.alpha),0.8)+P.speed_gain*(P.cruise_speed-o.speed);end
end
assert(all(isfinite([o.cmd,o.de,o.boost_flow,o.cruise_flow])),'Nonfinite command/flow');
if on,o.cmd=clip(o.cmd,P.cruise_idle_thrust,v.thrust_max);else,o.cmd=0;end
o.throttle=o.cmd/v.thrust_max;
if o.speed>=1e-8 && v.aero_enabled
    ph=a(11)*v.span/(2*max(o.speed,1e-3));qh=a(12)*v.chord/(2*max(o.speed,1e-3));rh=a(13)*v.span/(2*max(o.speed,1e-3));
    o.moment=qbar*v.area*[v.span*(v.roll_beta*o.beta+v.roll_p*ph+v.roll_r*rh); ...
        v.chord*(v.pitch0+v.pitch_alpha*o.alpha+v.pitch_q*qh+v.pitch_e*o.de); ...
        v.span*(v.yaw_beta*o.beta+v.yaw_p*ph+v.yaw_r*rh)];
end
o.quaternion_error=abs(norm(a(7:10))-1);
end

function da=rhs(t,s,P)
a=s.a;o=observe(t,s,P);v=P.vehicle;R=rotation(a(7:10));rates=a(11:13);m=v.mass_dry+a(16)+a(17);
da=zeros(17,1);da(1:3)=o.vn;
da(4:6)=(o.force+[o.total_thrust;0;0])/m+R'*[0;0;v.g]-cross(rates,a(4:6));
p=rates(1);q=rates(2);r=rates(3);da(7:10)=0.5*[0,-p,-q,-r;p,0,r,-q;q,-r,0,p;r,q,-p,0]*a(7:10);
da(11:13)=(o.moment-cross(rates,v.J.*rates))./v.J;
da(14)=-o.boost_flow-o.cruise_flow;
if s.cruise_ignited && ~s.cruise_exhausted,da(15)=(o.cmd-a(15))/v.thrust_tau;end
da(16)=-o.boost_flow;da(17)=-o.cruise_flow;assert(all(isfinite(da)),'Nonfinite derivative');
end

function out=step(t,dt,s,P)
if dt==0,out=s;return;end
assert(isfinite(dt)&&dt>0&&isfinite(t+dt),'Invalid step');k1=rhs(t,s,P);y=s;y.a=s.a+dt*k1/2;
k2=rhs(t+dt/2,y,P);y.a=s.a+dt*k2/2;k3=rhs(t+dt/2,y,P);y.a=s.a+dt*k3;k4=rhs(t+dt,y,P);
out=s;out.a=s.a+dt*(k1+2*k2+2*k3+k4)/6;out=normalize(out,P);
assert(out.a(15)>=-1e-9&&out.a(15)<=P.vehicle.thrust_max+1e-9,'Invalid thrust; reduce dt');
out.a(15)=clip(out.a(15),0,P.vehicle.thrust_max);
end

function s=normalize(s,P)
n=norm(s.a(7:10));assert(isfinite(n)&&n>=1e-12,'Invalid accepted quaternion');s.a(7:10)=s.a(7:10)/n;
s.a(14)=P.vehicle.mass_dry+s.a(16)+s.a(17);assert(all(isfinite(s.a))&&s.a(14)>0,'Invalid accepted state');
end

function yes=ready(s,P)
yes=-s.a(3)>=P.ignition_height && norm(s.a(4:6))>=P.ignition_speed;
end
function yes=crossed(kind,s,P)
if kind==1,yes=ready(s,P);elseif kind==4,yes=s.a(17)<=0;else,yes=s.a(3)>=0;end
end
function time=locate(t,dt,s,P,kind)
lo=0;hi=dt;
for k=1:55
    mid=(lo+hi)/2;probe=step(t,mid,s,P);
    if crossed(kind,probe,P),hi=mid;else,lo=mid;end
end
time=hi;
end

function [out,used,event]=advance(t,dt,s,P)
assert(all(isfinite(s.a))&&s.a(16)>=-1e-12&&s.a(17)>=-1e-12,'Invalid input inventory');
assert(s.a(15)>=0&&s.a(15)<=P.vehicle.thrust_max+1e-9,'Invalid input thrust');
assert(abs(s.a(14)-P.vehicle.mass_dry-s.a(16)-s.a(17))<=1e-9,'Inconsistent total mass');
if s.airborne&&s.a(3)>=0,out=s;out.a(3)=0;used=0;event=8;return;end
span=dt;times=inf(1,4);kinds=[1,2,4,8];
if s.boost_active&&P.burn_duration-t<=dt,span=max(P.burn_duration-t,0);times(2)=span;end
% Initial readiness must fire even if a later endpoint would lose a gate.
if ~s.cruise_ignited&&ready(s,P),span=0;times(1)=0;end
probe=step(t,span,s,P);
if ~s.cruise_ignited&&~isfinite(times(1))&&ready(probe,P)
    times(1)=locate(t,span,s,P,1);
end
if s.cruise_ignited&&~s.cruise_exhausted&&probe.a(17)<=0
    if s.a(17)<=0,times(3)=0;else,times(3)=locate(t,span,s,P,4);end
end
if span>0&&probe.a(3)>=0,times(4)=locate(t,span,s,P,8);end
used=min([span,times]);out=step(t,used,s,P);event=0;
for k=1:4,if isfinite(times(k))&&abs(times(k)-used)<1e-11,event=bitor(event,kinds(k));end,end
if bitand(event,8),out.a(3)=0;event=8;
else
    if bitand(event,1),out.cruise_ignited=true;out.ignition_time=t+used;end
    if bitand(event,2),out.boost_active=false;out.a(16)=0;end
    if bitand(event,4),out.cruise_exhausted=true;out.a(17)=0;out.a(15)=0;end
end
if out.a(3)<0,out.airborne=true;end
assert(out.a(16)>=-1e-10&&out.a(17)>=-1e-10,'Negative accepted inventory');out=normalize(out,P);
end

function row=sample(t,s,P,event)
a=s.a;o=observe(t,s,P);
row=[t,a(1),-a(3),a(2),o.vn(1),-o.vn(3),o.vn(2),a(1:13)',o.roll,o.pitch,o.yaw,o.alpha,o.beta, ...
    o.flight_path,o.psi_v,a(14),a(15),o.cmd,o.throttle,0,o.de,0,o.lift,o.drag,o.side,o.moment', ...
    o.quaternion_error,o.speed,o.boost_thrust,o.total_thrust,a(16),a(17),o.boost_flow,o.cruise_flow, ...
    s.boost_active,s.cruise_ignited,s.cruise_exhausted,o.stage,s.ignition_time,o.pitch_cmd,o.gamma_cmd, ...
    o.height_gate,o.speed_gate,event];
assert(numel(row)==58&&all(isfinite(row)),'Invalid CSV row');
end
function [rows,count]=append(rows,count,t,s,P,event)
if rows(count,1)==t,event=bitor(event,rows(count,58));else,count=count+1;end
rows(count,:)=sample(t,s,P,event);
end
function write_csv(path,headers,rows)
fid=fopen(path,'w');assert(fid>=0,'Cannot open CSV');cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',strjoin(headers,','));format=[repmat('%.17g,',1,numel(headers)-1),'%.17g\n'];
fprintf(fid,format,rows');[message,code]=ferror(fid);assert(code==0,'CSV write failed: %s',message);
end
function y=clip(x,lo,hi)
y=max(lo,min(hi,x));
end
