clc;
close all;

%% ------------------------------------------------------------------------
figure
subplot(3,2,1);
plot(Time', 180*xi_temp(3,:)/pi,'LineWidth', 2, 'Color', 'b');
%axis([0,120,-4,6])
hold on 
plot(Time', 180*ref_psi_Log/pi, 'LineWidth', 2, 'Color', 'r');
grid on
title('$\psi$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$\psi[deg]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$Actual$','$Reference$','Location','NorthEast');
set(l, 'interpreter', 'latex', 'fontsize', 18)

subplot(3,2,2);
% Plot change of yaw angle
plot(Time(1:end-1,1)', (180 / pi) * Logs(:,1),'LineWidth', 2, 'Color', 'b');
grid on
title('$\Delta{\psi}$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$\Delta{\psi}[deg]$','fontsize', 18, 'interpreter', 'latex')

% Plot Yaw Angle Graph
subplot(3,2,3);
% Plot Y position
plot(Time', xi_temp(2,:),'LineWidth', 2, 'Color', 'b');
%axis([0,120,-4,6])
hold on 
plot(Time', ref_Y_Log, 'LineWidth', 2, 'Color', 'r');
grid on
title('$Y$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$Y[m]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$Actual$','$Reference$','Location','NorthEast');
set(l, 'interpreter', 'latex', 'fontsize', 18)

subplot(3,2,4);
% Plot change of Y position
plot(Time(1:end-1,1)', Logs(:,2),'LineWidth', 2, 'Color', 'b');
grid on
title('$\Delta{Y}$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$\Delta{Y}[m]$','fontsize', 18, 'interpreter', 'latex')

% Plot input (Steering Angle)
subplot(3,2,5);
plot(dt * [1:1:length(u_star)], (180/pi) * u_star(1,:)) 
grid on
title('$Front\,\,steering\,\,angle$',...
      'fontsize', 18, 'interpreter', 'latex'); 
l = legend('$\delta_{f}$','$Reference$','Location','NorthEast');
set(l, 'interpreter', 'latex', 'fontsize', 18);
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex');
ylabel('$\delta_{f}[deg]$','fontsize', 18, 'interpreter', 'latex');

% Plot Trajectory
subplot(3,2,6);
plot(xi_temp(1,:), xi_temp(2,:),'LineWidth', 2, 'Color', 'b');
%axis([0,120,-4,6])
hold on 
plot(xi_temp(1,:), ref_Y_Log, 'LineWidth', 2, 'Color', 'r');
grid on
title('$Path\,\,following$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$X[m]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$Y[m]$','fontsize', 18, 'interpreter', 'latex')
l = legend('$Actual$','$Reference$','Location','NorthEast');
set(l, 'interpreter', 'latex', 'fontsize', 18)

%%-------------------------------------------------------------------------
figure
subplot(3,2,1);
% Plot computation time 
plot(dt * [1:1:length(u_star)], Logs(:,7),'LineWidth', 2, 'Color', 'b');
grid on
title('$YALMIP\,\,via\,\,fmincon\,\,computation\,\,time$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Computation\,\,Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$Simulation\,\,Time[s]$','fontsize', 18, 'interpreter', 'latex')

subplot(3,2,2);
% Plot Yaw Rare position
plot(Time', (180/pi) * xi_temp(5,:),'LineWidth', 2, 'Color', 'b');
grid on
title('$Yaw\,\,rate$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$r[deg/s]$','fontsize', 18, 'interpreter', 'latex')

subplot(3,2,3);
% Plot front slip angle
plot(dt * [1:1:length(u_star)], (180/pi)*Logs(:,3),'LineWidth', 2, 'Color', 'b');
grid on
title('$Front\,\,slip\,\,angle$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$\alpha_{f}[deg]$','fontsize', 18, 'interpreter', 'latex')

subplot(3,2,4);
% Plot rear slip angle
plot(dt * [1:1:length(u_star)], (180/pi)*Logs(:,4),'LineWidth', 2, 'Color', 'b');
grid on
title('$Rear\,\,slip\,\,angle$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$\alpha_{r}[deg]$','fontsize', 18, 'interpreter', 'latex')

subplot(3,2,5);
% Plot front tire corner force
plot(dt * [1:1:length(u_star)], Logs(:,5),'LineWidth', 2, 'Color', 'b');
grid on
title('$Front\,\,tire\,\,corner\,\,force$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$F_{y_f}[N]$','fontsize', 18, 'interpreter', 'latex')

subplot(3,2,6);
% Plot rear tire corner force
plot(dt * [1:1:length(u_star)], Logs(:,6),'LineWidth', 2, 'Color', 'b');
grid on
title('$Rear\,\,tire\,\,corner\,\,force$',...
      'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
ylabel('$F_{y_f}[N]$','fontsize', 18, 'interpreter', 'latex')

% % Plot Trajectory
% plot(Time', xi_temp(2,:),'LineWidth', 2, 'Color', 'b');
% %axis([0,120,-4,6])
% hold on 
% plot(Time', ref_Y_Log, 'LineWidth', 2, 'Color', 'r');
% grid on
% title('$Path\,\,following$',...
%       'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
% xlabel('$X[m]$','fontsize', 18, 'interpreter', 'latex')
% ylabel('$Y[m]$','fontsize', 18, 'interpreter', 'latex')
% l = legend('$Actual$','$Reference$','Location','NorthEast');
% set(l, 'interpreter', 'latex', 'fontsize', 18)

% 
% % Plot Yaw Angle Graph
% figure
% plot(Time', 180*xi_temp(3,:)/pi,'LineWidth', 2, 'Color', 'b');
% %axis([0,120,-4,6])
% hold on 
% plot(Time', 180*ref_psi_Log/pi, 'LineWidth', 2, 'Color', 'r');
% grid on
% title('$\psi$',...
%       'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
% xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
% ylabel('$\psi[deg]$','fontsize', 18, 'interpreter', 'latex')
% l = legend('$Actual$','$Reference$','Location','NorthEast');
% set(l, 'interpreter', 'latex', 'fontsize', 18)
% 
% 
% % Plot vy velocity
% figure
% plot(Time', xi_temp(4,:),'LineWidth', 2, 'Color', 'b');
% %axis([0,120,-4,6])
% grid on
% title('$Lateral\,\,Velocity$',...
%       'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
% xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
% ylabel('$v_y[m/s]$','fontsize', 18, 'interpreter', 'latex')
% 
% % Plot yaw rate velocity
% figure
% plot(Time', 180*xi_temp(5,:)/pi,'LineWidth', 2, 'Color', 'b');
% %axis([0,120,-4,6])
% grid on
% title('$Yaw\,\,rate$',...
%       'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
% xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
% ylabel('$\dot{\psi}=r\,\,[deg/s]$','fontsize', 18, 'interpreter', 'latex')
% 
% % Plot Corner Tire Forces at Front and Back
% figure
% subplot(2,1,1);
% plot(double(alphaF(1,:)), double(FyF(1,:)),'LineWidth', 2, 'Color', 'b');
% title('$Lateral(Corner)\,\,Tire\,\,Forces$',...
%       'fontsize',18, 'fontweight','b', 'interpreter', 'latex')
% ylabel('$F_{y_f}\,\,[N]$','fontsize', 18, 'interpreter', 'latex')
% subplot(2,1,2); 
% plot(double(alphaR(1,:)), double(FyR(1,:)),'LineWidth', 2, 'Color', 'b');
% ylabel('$F_{y_r}\,\,[N]$','fontsize', 18, 'interpreter', 'latex')
% xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex')
% 
% % PLOT INPUT (Steering Angle)
% figure
% plot(dt * [1:1:length(u_star)], (180/pi) * u_star(1,:)) 
% grid on
% title('Steerin angle at wheel side'); 
% l = legend('$\delta_{f}$','$Reference$','Location','NorthEast');
% set(l, 'interpreter', 'latex', 'fontsize', 18);
% xlabel('$Time[s]$','fontsize', 18, 'interpreter', 'latex');
% ylabel('$\delta_{f}[deg]$','fontsize', 18, 'interpreter', 'latex');