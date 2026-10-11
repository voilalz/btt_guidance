function plot_saved_boost_cruise(outputDir)
% Can plot either the delivered C CSV or new MATLAB results.
if nargin<1,outputDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');end
A=readtable(fullfile(outputDir,'boost_cruise.csv'));ign=A.ignition_time_s(end);
f=figure('Visible','off','Position',[100 100 1200 950]);
subplot(3,2,1);plot(A.x_m,A.y_m,'LineWidth',1.5);xlabel('Forward x (m)');ylabel('Height y (m)');grid on;
subplot(3,2,2);plot(A.t_s,A.y_m,'LineWidth',1.5);hold on;yline(40,'--');xlabel('Time (s)');ylabel('Height (m)');grid on;
subplot(3,2,3);plot(A.t_s,A.speed_mps,'LineWidth',1.5);hold on;yline(25,'--');xlabel('Time (s)');ylabel('Speed (m/s)');grid on;
subplot(3,2,4);plot(A.t_s,A.boost_thrust_N,A.t_s,A.thrust_N,A.t_s,A.total_thrust_N,'LineWidth',1.3);hold on;
plot(A.t_s,A.thrust_cmd_N,'--');xlabel('Time (s)');ylabel('Thrust (N)');grid on;legend('Solid','Cruise actual','Total','Cruise command');
subplot(3,2,5);plot(A.t_s,A.boost_fuel_kg,A.t_s,A.cruise_fuel_kg,'LineWidth',1.3);xlabel('Time (s)');ylabel('Remaining fuel (kg)');grid on;legend('Solid','Cruise');
subplot(3,2,6);stairs(A.t_s,A.stage,'LineWidth',1.5);xlabel('Time (s)');ylabel('Stage');ylim([-0.2 4.2]);yticks(0:4);yticklabels({'Boost','Overlap','Coast','Cruise','Empty'});grid on;
for j=2:6,subplot(3,2,j);if ign>=0,xline(ign,':');end;xline(4,'-.');end
print(f,fullfile(outputDir,'overview.png'),'-dpng','-r150');close(f);
f=figure('Visible','off','Position',[100 100 1150 750]);
subplot(2,2,1);plot(A.t_s,A.pitch_rad*180/pi,A.t_s,A.pitch_cmd_rad*180/pi,'--','LineWidth',1.3);xlabel('Time (s)');ylabel('Pitch (deg)');grid on;legend('Actual','Command');
subplot(2,2,2);plot(A.t_s,A.flight_path_rad*180/pi,A.t_s,A.gamma_cmd_rad*180/pi,'--','LineWidth',1.3);xlabel('Time (s)');ylabel('Flight path (deg)');grid on;legend('Actual','Command');
subplot(2,2,3);plot(A.t_s,A.alpha_rad*180/pi,A.t_s,A.delta_e_rad*180/pi,'LineWidth',1.3);xlabel('Time (s)');ylabel('Angle (deg)');grid on;legend('Alpha','Elevator');
subplot(2,2,4);plot(A.t_s,A.q_radps*180/pi,'LineWidth',1.3);xlabel('Time (s)');ylabel('Pitch rate (deg/s)');grid on;
print(f,fullfile(outputDir,'attitude.png'),'-dpng','-r150');close(f);
end
