function plot_saved_civil_route(outputDir,scene)
% Read saved C CSVs. Uses base MATLAB; this plotting entry does not simulate.
if nargin<1,outputDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');end
if nargin<2,scenes={'high','crossing','monitor'};else,scenes={char(scene)};end
for k=1:numel(scenes)
    path=fullfile(outputDir,[scenes{k},'.csv']);assert(exist(path,'file')==2,'CSV missing: %s',path);
    A=readtable(path);f=figure('Visible','off','Position',[100 100 1200 950]);
    subplot(3,2,1);plot(A.x_m,A.z_m,A.traffic_x_m,A.traffic_z_m,'LineWidth',1.4);
    xlabel('Forward x (m)');ylabel('Right z (m)');grid on;legend('Own aircraft','Traffic');title('Horizontal paths');
    subplot(3,2,2);plot(A.t_s,A.height_m,A.t_s,A.command_height_m,'--','LineWidth',1.4);
    xlabel('Time (s)');ylabel('Own height (m)');grid on;legend('Actual','Applied reference');title('Own altitude');
    subplot(3,2,3);plot(A.t_s,A.speed_mps,A.t_s,A.command_speed_mps,'--','LineWidth',1.4);
    xlabel('Time (s)');ylabel('Speed (m/s)');grid on;legend('Actual','Reference');
    subplot(3,2,4);plot(A.t_s,A.separation_m,'LineWidth',1.4);hold on;yline(80,'--');
    xlabel('Time (s)');ylabel('Actual separation (m)');grid on;title('Independent traffic separation');
    subplot(3,2,5);plot(A.t_s,A.boost_thrust_N,A.t_s,A.cruise_thrust_N,'LineWidth',1.4);
    xlabel('Time (s)');ylabel('Thrust (N)');grid on;legend('Solid boost','Cruise');
    subplot(3,2,6);stairs(A.t_s,A.phase,'LineWidth',1.4);hold on;stairs(A.t_s,A.safety_active,'--','LineWidth',1.4);
    xlabel('Time (s)');ylabel('Mode');grid on;legend('Phase: 0/1/2','Safety: 0/1');
    print(f,fullfile(outputDir,[scenes{k},'_matlab.png']),'-dpng','-r150');close(f);
end
end
