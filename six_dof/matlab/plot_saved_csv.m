function plot_saved_csv(outputDir)
% Plot C or MATLAB CSVs using only base MATLAB. CSV angles are radians.
if nargin<1
    outputDir=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results');
end
A=readtable(fullfile(outputDir,'constant.csv'));
B=readtable(fullfile(outputDir,'variable.csv'));
cases={A,B}; labels={'Constant command','Variable command'}; colors={[0.12 0.40 0.70],[0.85 0.30 0.10]};
f=figure('Visible','off','Position',[100 100 1000 700]); hold on;
for k=1:2, C=cases{k}; plot3(C.x_m,C.z_m,C.y_m,'Color',colors{k},'LineWidth',1.5); end
xlabel('Forward x (m)'); ylabel('Right z (m)'); zlabel('Height y (m)'); grid on; view(40,25);
legend(labels,'Location','best'); title('Six-DOF trajectory'); save_plot(f,outputDir,'trajectory_3d');
f=figure('Visible','off','Position',[100 100 1200 950]);
for k=1:2
    C=cases{k}; color=colors{k};
    subplot(3,2,1); hold on; plot(C.x_m,C.y_m,'Color',color,'LineWidth',1.4); xlabel('Forward x (m)'); ylabel('Height y (m)');
    subplot(3,2,2); hold on; plot(C.t_s,C.thrust_N,'Color',color); plot(C.t_s,C.thrust_cmd_N,'--','Color',color); xlabel('Time (s)'); ylabel('Thrust (N)');
    subplot(3,2,3); hold on; plot(C.t_s,sqrt(C.u_mps.^2+C.v_mps.^2+C.w_mps.^2),'Color',color); xlabel('Time (s)'); ylabel('Speed (m/s)');
    subplot(3,2,4); hold on; plot(C.t_s,C.mass_kg,'Color',color); xlabel('Time (s)'); ylabel('Mass (kg)');
    subplot(3,2,5); hold on; plot(C.t_s,C.alpha_rad*180/pi,'Color',color); plot(C.t_s,C.beta_rad*180/pi,'--','Color',color); xlabel('Time (s)'); ylabel('Alpha / beta (deg)');
    subplot(3,2,6); hold on; plot(C.t_s,C.throttle,'Color',color); xlabel('Time (s)'); ylabel('Throttle');
end
for j=1:6, subplot(3,2,j); grid on; end
subplot(3,2,1); legend(labels,'Location','best');
subplot(3,2,2); legend('Constant actual','Constant command','Variable actual','Variable command','Location','best');
subplot(3,2,5); legend('Constant alpha','Constant beta','Variable alpha','Variable beta','Location','best');
save_plot(f,outputDir,'overview');
f=figure('Visible','off','Position',[100 100 1200 950]);
angles={'roll_rad','pitch_rad','yaw_rad'}; rates={'p_radps','q_radps','r_radps'};
names={'Roll','Pitch','Yaw'};
for j=1:3
    for k=1:2
        C=cases{k}; subplot(3,2,2*j-1); hold on; plot(C.t_s,C.(angles{j})*180/pi,'Color',colors{k});
        xlabel('Time (s)'); ylabel([names{j} ' (deg)']); grid on;
        subplot(3,2,2*j); hold on; plot(C.t_s,C.(rates{j})*180/pi,'Color',colors{k});
        xlabel('Time (s)'); ylabel([rates{j}(1) ' (deg/s)']); grid on;
    end
end
subplot(3,2,1); legend(labels,'Location','best'); save_plot(f,outputDir,'attitude_rates');
f=figure('Visible','off','Position',[100 100 1000 750]); fields={'delta_a_rad','delta_e_rad','delta_r_rad'};
names={'Aileron','Elevator','Rudder'};
for j=1:3
    subplot(3,1,j); hold on;
    for k=1:2, C=cases{k}; plot(C.t_s,C.(fields{j})*180/pi,'Color',colors{k}); end
    xlabel('Time (s)'); ylabel([names{j} ' (deg)']); grid on;
end
subplot(3,1,1); legend(labels,'Location','best'); save_plot(f,outputDir,'controls');
end

function save_plot(f,outputDir,name)
print(f,fullfile(outputDir,[name '.png']),'-dpng','-r150'); close(f);
end
