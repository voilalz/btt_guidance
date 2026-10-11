function results = run_variable_thrust_demo(output_dir, dt, t_end)
%RUN_VARIABLE_THRUST_DEMO 连续可变推力三维轨迹教学仿真。
% 在本文件所在目录运行：results = run_variable_thrust_demo;
% 可选：results = run_variable_thrust_demo('my_results', 0.02, 120);
% 不需要航空航天、控制或优化工具箱；只用MATLAB基础函数。
% 状态：[x;y;z;vx;vy;vz;m;T]，SI单位，x前/y上/z右。
% 本环境未安装MATLAB/Octave。本源文件未做MATLAB运行验证。

if nargin < 1 || isempty(output_dir)
    output_dir = fullfile(fileparts(mfilename('fullpath')), 'results_matlab');
end
if nargin < 2, dt = 0.02; end
if nargin < 3, t_end = 120; end
if ~isnumeric(dt) || ~isscalar(dt) || ~isreal(dt) || ~isfinite(dt) || dt <= 0 || dt > 1
    error('dt必须是0<dt<=1的有限实数，单位s。');
end
if ~isnumeric(t_end) || ~isscalar(t_end) || ~isreal(t_end) || ~isfinite(t_end) ...
        || t_end <= 0 || t_end > 3600 || t_end/dt > 1000000
    error('时长必须满足0<t_end<=3600 s且总步数不超过1000000。');
end
if ~exist(output_dir, 'dir')
    [ok,msg] = mkdir(output_dir);
    if ~ok, error('无法创建输出目录：%s',msg); end
end
p = default_params();
results.constant = simulate(p, 0, dt, t_end);
results.variable = simulate(p, 1, dt, t_end);
write_csv(fullfile(output_dir,'constant.csv'),results.constant);
write_csv(fullfile(output_dir,'variable.csv'),results.variable);
plot_results(results, output_dir);
for mode = 0:1
    if mode == 0, name = 'constant'; else, name = 'variable'; end
    a = results.(name);
    fprintf('%s: t=%.6f s, x=%.3f m, y=%.3f m, z=%.3f m\n', ...
        name,a(end,1),a(end,2),a(end,3),a(end,4));
    fprintf('  speed=%.6f m/s, max_height=%.3f m, fuel_used=%.9f kg\n', ...
        a(end,8),max(a(:,3)),p.mass0-a(end,13));
end
fprintf('CSV和图像已写入：%s\n',output_dir);
end

function p = default_params()
% 与c/trajectory.c完全相同的虚构参数；此处是MATLAB的参数入口。
p.g=9.80665; p.rho0=1.225; p.scale_height=8500; p.area=0.8;
p.cl0=0.2; p.cl_alpha=4.5; p.cl_min=-0.6; p.cl_max=1.2;
p.cd0=0.035; p.drag_k=0.06;
p.mass0=12; p.mass_dry=11.7; p.thrust_max=60; p.thrust_tau=1;
p.fuel_c0=5e-5; p.fuel_cT=1.5e-5; p.consume_fuel=true;
p.speed0=25; p.height0=200;
p.alpha_amp=0.25*pi/180; p.alpha_period=30;
p.bank_amp=8*pi/180; p.bank_period=60;
p.throttle_mean=0.2; p.throttle_amp1=0.15; p.throttle_period1=40;
p.throttle_amp2=0.05; p.throttle_period2=15;
% 只平衡初始竖直力，攻角不是控制器算出的。
rho=p.rho0*exp(-p.height0/p.scale_height);
qs=0.5*rho*p.speed0^2*p.area; thrust=p.thrust_max*p.throttle_mean;
alpha=(p.mass0*p.g/qs-p.cl0)/p.cl_alpha;
for j=1:8
    residual=qs*(p.cl0+p.cl_alpha*alpha)+thrust*sin(alpha)-p.mass0*p.g;
    alpha=alpha-residual/(qs*p.cl_alpha+thrust*cos(alpha));
end
p.alpha_trim=alpha;
end

function on = engine_on(s,p)
on = ~p.consume_fuel || s(7)>p.mass_dry+1e-12;
end

function o = observe(t,s,p,mode,on)
% 密度、气动、姿态代数输入和动力状态，供积分及输出共同使用。
horizontal=hypot(s(4),s(6));
o.speed=hypot(horizontal,s(5));
o.theta=atan2(s(5),horizontal); o.psi_v=atan2(-s(6),s(4));
o.alpha=p.alpha_trim+p.alpha_amp*sin(2*pi*t/p.alpha_period);
o.bank=p.bank_amp*sin(2*pi*t/p.bank_period);
o.rho=p.rho0*exp(-max(s(2),0)/p.scale_height);
cl=min(p.cl_max,max(p.cl_min,p.cl0+p.cl_alpha*o.alpha));
cd=p.cd0+p.drag_k*cl^2; qs=0.5*o.rho*o.speed^2*p.area;
o.lift=qs*cl; o.drag=qs*cd;
o.throttle=p.throttle_mean;
if mode == 1
    o.throttle=o.throttle+p.throttle_amp1*sin(2*pi*t/p.throttle_period1) ...
        +p.throttle_amp2*sin(2*pi*t/p.throttle_period2);
end
o.throttle=max(0,min(1,o.throttle));
o.thrust_cmd=0; o.thrust=0; o.fuel_flow=0;
if on
    o.thrust_cmd=p.thrust_max*o.throttle;
    o.thrust=max(0,min(p.thrust_max,s(8)));
    if p.consume_fuel, o.fuel_flow=p.fuel_c0+p.fuel_cT*o.thrust; end
end
end

function ds = rhs(t,s,p,mode,on)
o=observe(t,s,p,mode,on);
if o.speed < 1e-9, ev=[1;0;0]; else, ev=s(4:6)/o.speed; end
% 将惯性系的竖直方向投影到与速度正交的平面。
reference=[0;1;0]; up=reference-dot(reference,ev)*ev;
if norm(up)<1e-8
    % 近竖直时参考x轴；这是有限性降级，不是姿态动力学。
    reference=[1;0;0]; up=reference-dot(reference,ev)*ev;
end
up=up/norm(up); side=cross(ev,up);
en=cos(o.bank)*up+sin(o.bank)*side;
force=(o.thrust*cos(o.alpha)-o.drag)*ev+(o.thrust*sin(o.alpha)+o.lift)*en;
mass=max(s(7),p.mass_dry);
ds=zeros(8,1); ds(1:3)=s(4:6);
ds(4:6)=force/mass+[0;-p.g;0]; ds(7)=-o.fuel_flow;
if on, ds(8)=(o.thrust_cmd-s(8))/p.thrust_tau; end
end

function out = rk4(t,h,start,p,mode,on)
% 经典四阶Runge-Kutta；指令在每个中间时刻重新计算。
k1=rhs(t,start,p,mode,on);
k2=rhs(t+h/2,start+h*k1/2,p,mode,on);
k3=rhs(t+h/2,start+h*k2/2,p,mode,on);
k4=rhs(t+h,start+h*k3,p,mode,on);
out=start+h*(k1+2*k2+2*k3+k4)/6;
end

function next = step(t,h,start,p,mode)
on=engine_on(start,p);
if ~on, start(7)=p.mass_dry; start(8)=0; end
next=rk4(t,h,start,p,mode,on);
if on && p.consume_fuel && next(7)<=p.mass_dry
    % 先积分至耗尽时刻，然后熄火并积分该步的剩余部分。
    lo=0; hi=h;
    for j=1:45
        mid=(lo+hi)/2; probe=rk4(t,mid,start,p,mode,true);
        if probe(7)>p.mass_dry, lo=mid; else, hi=mid; end
    end
    burn_time=(lo+hi)/2;
    probe=rk4(t,burn_time,start,p,mode,true);
    probe(7)=p.mass_dry; probe(8)=0;
    next=rk4(t+burn_time,h-burn_time,probe,p,mode,false);
end
next(7)=max(next(7),p.mass_dry);
next(8)=max(0,min(p.thrust_max,next(8)));
end

function [next,used,hit] = advance(t,h,start,p,mode)
used=h; hit=false;
if start(2)<=0, next=start; next(2)=0; used=0; hit=true; return; end
next=step(t,h,start,p,mode);
if next(2)>0, return; end
lo=0; hi=h;
for j=1:45
    mid=(lo+hi)/2; probe=step(t,mid,start,p,mode);
    if probe(2)>0, lo=mid; else, hi=mid; end
end
used=(lo+hi)/2; next=step(t,used,start,p,mode); next(2)=0; hit=true;
end

function a = simulate(p,mode,dt,t_end)
s=zeros(8,1); s(2)=p.height0; s(4)=p.speed0;
s(7)=p.mass0; s(8)=p.thrust_max*p.throttle_mean;
if ~engine_on(s,p), s(8)=0; end
a=zeros(ceil(t_end/dt)+2,20); t=0; n=1;
a(n,:)=output_row(t,s,p,mode);
while t<t_end-t_end*1e-12
    h=min(dt,t_end-t);
    [s,used,hit]=advance(t,h,s,p,mode); t=t+used;
    if any(~isfinite(s)), error('仿真状态出现NaN/Inf，请核查参数和步长。'); end
    n=n+1; a(n,:)=output_row(t,s,p,mode);
    if hit, break; end
end
a=a(1:n,:);
end

function row = output_row(t,s,p,mode)
o=observe(t,s,p,mode,engine_on(s,p)); degree=180/pi;
row=[t,s(1:6)',o.speed,o.theta*degree,o.psi_v*degree,o.alpha*degree, ...
    o.bank*degree,s(7),o.throttle,o.thrust_cmd,o.thrust,o.lift,o.drag,o.rho,o.fuel_flow];
end

function write_csv(filename,a)
fid=fopen(filename,'w');
if fid<0, error('无法创建CSV：%s',filename); end
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,['t_s,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,speed_mps,theta_deg,' ...
    'psi_v_deg,alpha_deg,bank_deg,mass_kg,throttle,thrust_cmd_N,thrust_N,' ...
    'lift_N,drag_N,rho_kgm3,fuel_flow_kgps\n']);
format=[repmat('%.17g,',1,19),'%.17g\n'];
fprintf(fid,format,a');
end

function plot_results(r,output_dir)
% 颜色一致：蓝色恒定推力，红色连续变推力。
a=r.constant; b=r.variable; blue=[0.12,0.40,0.75]; red=[0.85,0.27,0.17];
fig1=figure('Color','w','Name','Variable thrust: trajectory and states', ...
    'Position',[50,50,1250,740]);
subplot(2,3,1); plot(a(:,2),a(:,3),'Color',blue,'LineWidth',1.3); hold on;
plot(b(:,2),b(:,3),'Color',red,'LineWidth',1.3); grid on;
xlabel('x [m]'); ylabel('Height y [m]'); title('Vertical projection');
legend('Constant','Variable','Location','best');
subplot(2,3,2); plot(a(:,2),a(:,4),'Color',blue,'LineWidth',1.3); hold on;
plot(b(:,2),b(:,4),'Color',red,'LineWidth',1.3); grid on; axis equal;
xlabel('x [m]'); ylabel('z [m]'); title('Horizontal projection');
subplot(2,3,3); plot(a(:,1),a(:,8),'Color',blue,'LineWidth',1.3); hold on;
plot(b(:,1),b(:,8),'Color',red,'LineWidth',1.3); grid on;
xlabel('Time [s]'); ylabel('Speed [m/s]'); title('Speed response');
subplot(2,3,4); plot(a(:,1),a(:,3),'Color',blue,'LineWidth',1.3); hold on;
plot(b(:,1),b(:,3),'Color',red,'LineWidth',1.3); grid on;
xlabel('Time [s]'); ylabel('Height [m]'); title('Height');
subplot(2,3,5); plot(a(:,1),a(:,16),'Color',blue,'LineWidth',1.3); hold on;
plot(b(:,1),b(:,15),'--','Color',[0.6,0.6,0.6],'LineWidth',1);
plot(b(:,1),b(:,16),'Color',red,'LineWidth',1.3); grid on;
xlabel('Time [s]'); ylabel('Thrust [N]'); title('Command and actual thrust');
legend('Constant actual','Variable command','Variable actual','Location','best');
subplot(2,3,6); plot(a(:,1),a(:,13),'Color',blue,'LineWidth',1.3); hold on;
plot(b(:,1),b(:,13),'Color',red,'LineWidth',1.3); grid on;
xlabel('Time [s]'); ylabel('Mass [kg]'); title('Fuel consumption');
print(fig1,fullfile(output_dir,'overview.png'),'-dpng','-r160');

fig2=figure('Color','w','Name','3D trajectory','Position',[80,80,1000,660]);
% plot3的显示竖轴用y，所以实参顺序为x,z,y，而不是改坐标定义。
plot3(a(:,2),a(:,4),a(:,3),'Color',blue,'LineWidth',1.5); hold on;
plot3(b(:,2),b(:,4),b(:,3),'Color',red,'LineWidth',1.5);
grid on; view(35,25); xlabel('x [m]'); ylabel('z [m]'); zlabel('Height y [m]');
legend('Constant','Variable','Location','best'); title('3D trajectory (vertical scale enlarged)');
print(fig2,fullfile(output_dir,'trajectory_3d.png'),'-dpng','-r160');
end
