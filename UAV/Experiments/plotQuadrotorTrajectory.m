% This script plots the closed-loop responses of the nonlinear MPC
% controller used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Plot the closed-loop response.
time = 0:Ts:Duration;
time  = time (:,1:end-p);
yreftot = QuadrotorReferenceTrajectory(time,kk)';
%     yreftot(60:80+18,2) =  yreftot(60,2);
% Plot the states.
figure('Name','States')

subplot(2,3,1)
hold on
plot(time,xHistory(:,1))
plot(time,yreftot(:,1))
grid on
xlabel('time')
ylabel('x')
legend('actual','reference','Location','southeast')
title('Qruadrotor x position')

subplot(2,3,2)
hold on
plot(time,xHistory(:,2))
plot(time,yreftot(:,2))
grid on
xlabel('time')
ylabel('y')
legend('actual','reference','Location','southeast')
title('Qruadrotor y position')

subplot(2,3,3)
hold on
plot(time,xHistory(:,3))
plot(time,yreftot(:,3))
grid on
xlabel('time')
ylabel('z')
legend('actual','reference','Location','southeast')
title('Qruadrotor z position')

subplot(2,3,4)
hold on
plot(time,xHistory(:,4))
plot(time,yreftot(:,4))
grid on
xlabel('time')
ylabel('phi')
legend('actual','reference','Location','southeast')
title('Qruadrotor phi angle')

subplot(2,3,5)
hold on
plot(time,xHistory(:,5))
plot(time,yreftot(:,5))
grid on
xlabel('time')
ylabel('theta')
legend('actual','reference','Location','southeast')
title('Qruadrotor theta angle')

subplot(2,3,6)
hold on
plot(time,xHistory(:,6))
plot(time,yreftot(:,6))
grid on
xlabel('time')
ylabel('psi')
legend('actual','reference','Location','southeast')
title('Qruadrotor psi angle')

% % Plot the manipulated variables.
% figure('Name','Control Inputs')
% 
% subplot(2,2,1)
% hold on
% stairs(time,uHistory(:,1))
% ylim([-0.5,12.5])
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% legend('actual','reference')
% title('Input 1')
% 
% subplot(2,2,2)
% hold on
% stairs(time,uHistory(:,2))
% ylim([-0.5,12.5])
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% title('Input 2')
% legend('actual','reference')
% 
% subplot(2,2,3)
% hold on
% stairs(time,uHistory(:,3))
% ylim([-0.5,12.5])
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% title('Input 3')
% legend('actual','reference')
% 
% subplot(2,2,4)
% hold on
% stairs(time,uHistory(:,4))
% ylim([-0.5,12.5])
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% title('Input 4')
% legend('actual','reference')


% % Plot the manipulated variables.
% figure('Name','Control Inputs changes')
% 
% subplot(2,2,1)
% hold on
% stairs(time,duHistory(:,1))
% % plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% legend('actual','reference')
% title('Input 1')
% 
% subplot(2,2,2)
% hold on
% stairs(time,duHistory(:,2))
% % plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% title('Input 2')
% legend('actual','reference')
% 
% subplot(2,2,3)
% hold on
% stairs(time,duHistory(:,3))
% % plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% title('Input 3')
% legend('actual','reference')
% 
% subplot(2,2,4)
% hold on
% stairs(time,duHistory(:,4))
% % plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
% grid on
% xlabel('time')
% title('Input 4')
% legend('actual','reference')
% 
% 
