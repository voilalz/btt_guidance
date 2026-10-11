function summary = run_six_dof_demo(outputDir, duration, dt, outputStride)
% Independent 15-state six-DOF teaching simulation (SI units, radians).
% Internal: N,E,D,u,v,w,a,b,c,d,p,q,r,m,T. Output: x=N,y=-D,z=E.
% Example: run_six_dof_demo(); No toolbox required.
if nargin < 1 || isempty(outputDir)
    outputDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results_matlab');
end
if nargin < 2, duration = 120; end
if nargin < 3, dt = 0.01; end
if nargin < 4, outputStride = 5; end
assert(isscalar(duration) && isfinite(duration) && duration > 0, 'Invalid duration');
assert(isscalar(dt) && isfinite(dt) && dt > 0, 'Invalid dt');
assert(isscalar(outputStride) && isfinite(outputStride) && outputStride >= 1 && outputStride == floor(outputStride), 'Invalid output stride');
if ~exist(outputDir,'dir'), mkdir(outputDir); end
P = default_params(); [P, initial] = initial_trim(P);
headers = {'t_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','N_m','E_m','D_m', ...
    'u_mps','v_mps','w_mps','quat_a','quat_b','quat_c','quat_d','p_radps','q_radps','r_radps', ...
    'roll_rad','pitch_rad','yaw_rad','alpha_rad','beta_rad','flight_path_rad','psi_v_rad', ...
    'mass_kg','thrust_N','thrust_cmd_N','throttle','delta_a_rad','delta_e_rad','delta_r_rad', ...
    'lift_N','drag_N','side_N','Mx_Nm','My_Nm','Mz_Nm','quat_norm_error'};
summary.parameters = P; summary.duration_requested_s = duration;
summary.dt_s = dt; summary.output_stride = outputStride;
modes = {'constant','variable'};
for modeIndex = 1:2
    mode = modes{modeIndex}; s = initial; t = 0; steps = 0; count = 1; ground = false;
    rows = zeros(floor(ceil(duration/dt)/outputStride)+3, numel(headers));
    rows(count,:) = sample(t,s,P,mode); last = 0;
    while t < duration && ~ground
        next = min(duration,(steps+1)*dt);
        assert(next > t, 'Time step cannot advance floating-point time');
        [s,used,ground] = advance(t,next-t,s,P,mode);
        if ground, t = t+used; else, t = next; end
        steps = steps+1;
        if mod(steps,outputStride) == 0 || ground || t == duration
            if t > last
                count = count+1; rows(count,:) = sample(t,s,P,mode); last = t;
            end
        end
    end
    rows = rows(1:count,:); csv = fullfile(outputDir,[mode '.csv']);
    write_csv(csv,headers,rows);
    item.samples = count; item.end_time_s = t; item.final_position_m = [s(1),-s(3),s(2)];
    item.final_mass_kg = s(14); item.max_quaternion_norm_error = max(rows(:,41));
    if ground, item.termination = 'ground'; else, item.termination = 'duration'; end
    summary.(mode) = item;
    fprintf('%s: %d samples, end=%.12g s (%s)\n',mode,count,t,item.termination);
end
save(fullfile(outputDir,'summary.mat'),'summary');
plot_saved_csv(outputDir);
end

function P = default_params()
P.g=9.80665; P.rho0=1.225; P.scale_height=8500;
P.area=0.8; P.span=2; P.chord=0.4; P.J=[1;1.6;2.2];
P.cl0=0.2; P.cl_alpha=4.5; P.cl_min=-0.6; P.cl_max=1.2;
P.cd0=0.035; P.drag_k=0.06; P.cy_beta=-0.5; P.cy_r=0.12;
P.roll_beta=-0.08; P.roll_p=-0.45; P.roll_r=0.10; P.roll_a=0.12;
P.pitch0=0.03; P.pitch_alpha=-0.80; P.pitch_q=-5; P.pitch_e=-1;
P.yaw_beta=0.08; P.yaw_p=-0.04; P.yaw_r=-0.20; P.yaw_rudder=-0.08;
P.mass0=12; P.mass_dry=11.7; P.thrust_max=60; P.thrust_tau=1;
P.fuel_c0=5e-5; P.fuel_cT=1.5e-5; P.consume_fuel=true;
P.speed0=25; P.height0=200; P.aero_enabled=true; P.disturbances=true;
P.throttle_override=-1;
end

function [P,s] = initial_trim(P)
% P is immutable during integration; reject bad edited defaults before clipping.
values=struct2cell(P);
assert(all(cellfun(@(value) all(isfinite(value(:))),values)), 'Nonfinite model parameter');
lo=-15*pi/180; hi=15*pi/180; flo=trim_residual(lo,P); fhi=trim_residual(hi,P);
assert(isfinite(flo) && isfinite(fhi) && flo*fhi <= 0,'No trim root in +/-15 degrees');
for k=1:80
    mid=(lo+hi)/2; fm=trim_residual(mid,P);
    assert(isfinite(fm),'Nonfinite trim residual');
    if abs(fm)<1e-13, break; end
    if (fm>0)==(flo>0), lo=mid; flo=fm; else, hi=mid; end
end
P.alpha_trim=mid; rho=P.rho0*exp(-P.height0/P.scale_height);
CL=clip(P.cl0+P.cl_alpha*mid,P.cl_min,P.cl_max);
drag=0.5*rho*P.speed0^2*P.area*(P.cd0+P.drag_k*CL^2);
assert(abs(cos(mid))>1e-6 && abs(P.pitch_e)>1e-12,'Singular trim');
T=drag/cos(mid); assert(isfinite(T) && T>=0 && T<=P.thrust_max,'Trim thrust outside limits');
P.throttle_trim=T/P.thrust_max; P.elevator_trim=-(P.pitch0+P.pitch_alpha*mid)/P.pitch_e;
assert(all(isfinite([P.alpha_trim,P.throttle_trim,P.elevator_trim])), 'Nonfinite trim output');
s=zeros(15,1); s(3)=-P.height0; s(4)=P.speed0*cos(mid); s(6)=P.speed0*sin(mid);
s(7)=cos(mid/2); s(9)=sin(mid/2); s(14)=P.mass0; s(15)=T;
end

function f = trim_residual(alpha,P)
rho=P.rho0*exp(-max(P.height0,0)/P.scale_height); qs=0.5*rho*P.speed0^2*P.area;
CL=clip(P.cl0+P.cl_alpha*alpha,P.cl_min,P.cl_max);
f=qs*CL+qs*(P.cd0+P.drag_k*CL^2)*tan(alpha)-P.mass0*P.g;
end

function R = rotation(q)
n=norm(q); assert(isfinite(n) && n>=1e-12,'Invalid quaternion');
q=q/n; a=q(1); b=q(2); c=q(3); d=q(4);
R=[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c); ...
   2*(b*c+a*d), a*a-b*b+c*c-d*d, 2*(c*d-a*b); ...
   2*(b*d-a*c), 2*(c*d+a*b), a*a-b*b-c*c+d*d];
end

function o = observe(t,s,P,mode,engineOn)
assert(all(isfinite(s)) && s(14)>0,'Invalid state');
R=rotation(s(7:10)); q=s(7:10)/norm(s(7:10));
a=q(1); b=q(2); c=q(3); d=q(4);
o.roll=atan2(2*(a*b+c*d),1-2*(b*b+c*c));
o.pitch=asin(clip(2*(a*c-d*b),-1,1));
o.yaw=atan2(2*(a*d+b*c),1-2*(c*c+d*d));
o.quaternion_error=abs(norm(s(7:10))-1);
o.vn=R*s(4:6); o.speed=norm(s(4:6));
o.flight_path=atan2(-o.vn(3),hypot(o.vn(1),o.vn(2)));
o.psi_v=atan2(-o.vn(2),o.vn(1));
o.rho=P.rho0*exp(-max(-s(3),0)/P.scale_height);
o.da=0; o.de=P.elevator_trim; o.dr=0;
if P.disturbances
    o.da=0.2*pi/180*window(t,20,40)*sin(2*pi*(t-20)/8);
    o.dr=0.1*pi/180*window(t,45,65)*sin(2*pi*(t-45)/10);
end
o.throttle=P.throttle_trim;
if P.throttle_override>=0, o.throttle=P.throttle_override;
elseif strcmp(mode,'variable'), o.throttle=o.throttle+0.15*sin(2*pi*t/40)+0.05*sin(2*pi*t/15);
end
o.throttle=clip(o.throttle,0,1); o.thrust_cmd=0; o.fuel_flow=0;
if engineOn
    o.thrust_cmd=P.thrust_max*o.throttle;
    if P.consume_fuel, o.fuel_flow=P.fuel_c0+P.fuel_cT*s(15); end
end
o.alpha=0; o.beta=0; o.force=zeros(3,1); o.moment=zeros(3,1);
o.lift=0; o.drag=0; o.side=0;
if o.speed>=1e-8
    o.alpha=atan2(s(6),s(4)); o.beta=atan2(s(5),hypot(s(4),s(6)));
    if P.aero_enabled
        qbar=0.5*o.rho*o.speed^2; CL=clip(P.cl0+P.cl_alpha*o.alpha,P.cl_min,P.cl_max);
        CD=P.cd0+P.drag_k*CL^2; CY=P.cy_beta*o.beta+P.cy_r*o.dr;
        o.lift=qbar*P.area*CL; o.drag=qbar*P.area*CD; o.side=qbar*P.area*CY;
        ca=cos(o.alpha); sa=sin(o.alpha); cb=cos(o.beta); sb=sin(o.beta);
        ex=[ca*cb;sb;sa*cb]; ey=[-ca*sb;cb;-sa*sb]; ez=[-sa;0;ca];
        o.force=-o.drag*ex+o.side*ey-o.lift*ez;
        ph=s(11)*P.span/(2*max(o.speed,1e-3));
        qh=s(12)*P.chord/(2*max(o.speed,1e-3));
        rh=s(13)*P.span/(2*max(o.speed,1e-3));
        o.moment=qbar*P.area*[P.span*(P.roll_beta*o.beta+P.roll_p*ph+P.roll_r*rh+P.roll_a*o.da); ...
            P.chord*(P.pitch0+P.pitch_alpha*o.alpha+P.pitch_q*qh+P.pitch_e*o.de); ...
            P.span*(P.yaw_beta*o.beta+P.yaw_p*ph+P.yaw_r*rh+P.yaw_rudder*o.dr)];
    end
end
assert(all(isfinite([o.force;o.moment;o.vn])),'Nonfinite forces or velocity');
end

function ds = rhs(t,s,P,mode,engineOn)
o=observe(t,s,P,mode,engineOn); R=rotation(s(7:10)); rates=s(11:13);
ds=zeros(15,1); ds(1:3)=o.vn;
F=o.force; if engineOn, F(1)=F(1)+s(15); end
ds(4:6)=F/s(14)+R'*[0;0;P.g]-cross(rates,s(4:6));
p=rates(1); q=rates(2); r=rates(3);
ds(7:10)=0.5*[0,-p,-q,-r;p,0,r,-q;q,-r,0,p;r,q,-p,0]*s(7:10);
ds(11:13)=(o.moment-cross(rates,P.J.*rates))./P.J;
if engineOn
    ds(14)=-o.fuel_flow; ds(15)=(o.thrust_cmd-s(15))/P.thrust_tau;
end
assert(all(isfinite(ds)),'Nonfinite derivative');
end

function out = rk4(t,dt,s,P,mode,engineOn)
assert(isfinite(dt) && dt>0 && isfinite(t+dt),'Invalid RK4 step');
k1=rhs(t,s,P,mode,engineOn); k2=rhs(t+dt/2,s+dt*k1/2,P,mode,engineOn);
k3=rhs(t+dt/2,s+dt*k2/2,P,mode,engineOn); k4=rhs(t+dt,s+dt*k3,P,mode,engineOn);
out=s+dt*(k1+2*k2+2*k3+k4)/6;
assert(all(isfinite(out)) && out(14)>0,'Nonfinite/invalid accepted state');
n=norm(out(7:10)); assert(isfinite(n) && n>=1e-12,'Invalid accepted quaternion');
out(7:10)=out(7:10)/n;
assert(out(15)>=-1e-9 && out(15)<=P.thrust_max+1e-9,'Invalid accepted thrust; reduce dt');
out(15)=clip(out(15),0,P.thrust_max);
end

function on = engine_running(s,P)
on=~P.consume_fuel || s(14)>P.mass_dry;
end

function out = fuel_step(t,dt,s,P,mode)
on=engine_running(s,P);
if ~on, s(14)=P.mass_dry; s(15)=0; end
out=rk4(t,dt,s,P,mode,on);
if ~on || ~P.consume_fuel || out(14)>P.mass_dry, return; end
lo=0; hi=dt;
for k=1:60
    mid=(lo+hi)/2; probe=rk4(t,mid,s,P,mode,true);
    if probe(14)>P.mass_dry, lo=mid; else, hi=mid; end
end
out=s; if hi>0, out=rk4(t,hi,out,P,mode,true); end
out(14)=P.mass_dry; out(15)=0;
if dt>hi, out=rk4(t+hi,dt-hi,out,P,mode,false); end
end

function [out,used,ground] = advance(t,dt,s,P,mode)
assert(all(isfinite(s)) && s(14)>=P.mass_dry-1e-10 && s(15)>=0 && s(15)<=P.thrust_max+1e-9,'Invalid input state');
n=norm(s(7:10)); assert(isfinite(n) && n>=1e-12,'Invalid input quaternion'); s(7:10)=s(7:10)/n;
if s(14)<=P.mass_dry && P.consume_fuel, s(14)=P.mass_dry; s(15)=0; end
if s(3)>=0, out=s; out(3)=0; used=0; ground=true; return; end
out=fuel_step(t,dt,s,P,mode); ground=out(3)>=0; used=dt;
if ground
    lo=0; hi=dt;
    for k=1:60
        mid=(lo+hi)/2; probe=fuel_step(t,mid,s,P,mode);
        if probe(3)<0, lo=mid; else, hi=mid; end
    end
    out=fuel_step(t,hi,s,P,mode); out(3)=0; used=hi;
end
assert(out(14)>=P.mass_dry-1e-10,'Mass below dry mass');
end

function row = sample(t,s,P,mode)
o=observe(t,s,P,mode,engine_running(s,P));
row=[t,s(1),-s(3),s(2),o.vn(1),-o.vn(3),o.vn(2),s(1:13)', ...
    o.roll,o.pitch,o.yaw,o.alpha,o.beta,o.flight_path,o.psi_v,s(14),s(15), ...
    o.thrust_cmd,o.throttle,o.da,o.de,o.dr,o.lift,o.drag,o.side,o.moment',o.quaternion_error];
assert(numel(row)==41 && all(isfinite(row)),'Invalid CSV sample');
end

function write_csv(path,headers,rows)
fid=fopen(path,'w'); assert(fid>=0,'Cannot open output CSV'); cleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',strjoin(headers,',')); format=[repmat('%.17g,',1,numel(headers)-1) '%.17g\n'];
fprintf(fid,format,rows'); [message,errorCode]=ferror(fid);
assert(errorCode==0,'CSV write failed: %s',message); %#ok<NASGU>
end

function y = window(t,t1,t2)
if t<=t1 || t>=t2, y=0; else, y=sin(pi*(t-t1)/(t2-t1))^2; end
end
function y = clip(x,lo,hi)
y=max(lo,min(hi,x));
end
