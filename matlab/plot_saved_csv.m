function plot_saved_csv(results_dir)
%PLOT_SAVED_CSV 在MATLAB直接查看随包交付的、已由C计算的轨迹。
% plot_saved_csv; 或 plot_saved_csv('C程序生成CSV所在目录');
if nargin<1
    results_dir=fullfile(fileparts(mfilename('fullpath')),'..','results');
end
a=dlmread(fullfile(results_dir,'constant.csv'),',',1,0);
b=dlmread(fullfile(results_dir,'variable.csv'),',',1,0);
figure('Color','w');
subplot(2,2,1); plot(a(:,2),a(:,3),'b-',b(:,2),b(:,3),'r-'); grid on;
xlabel('x [m]'); ylabel('Height y [m]'); legend('Constant','Variable');
subplot(2,2,2); plot3(a(:,2),a(:,4),a(:,3),'b-',b(:,2),b(:,4),b(:,3),'r-'); grid on;
xlabel('x [m]'); ylabel('z [m]'); zlabel('Height y [m]');
subplot(2,2,3); plot(a(:,1),a(:,8),'b-',b(:,1),b(:,8),'r-'); grid on;
xlabel('Time [s]'); ylabel('Speed [m/s]');
subplot(2,2,4); plot(b(:,1),b(:,15),'k--',b(:,1),b(:,16),'r-'); grid on;
xlabel('Time [s]'); ylabel('Thrust [N]'); legend('Command','Actual');
end
