% This script plots the closed-loop responses of the nonlinear MPC
% controller used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Plot the closed-loop response.
tim = 0:Ts:Duration;
time = tim(1:length(xHistory))';
t = 0:Ts:Duration;
yreftot = VECReferenceTrajectory(Ts,t,f,14,10,2,.1,0,100)';

yreftot =yreftot';
% Plot the states.
figure('Name','States')

subplot(2,2,1)
hold on
plot(time,xHistory(:,1))
plot(time,yreftot(:,1))
grid on
xlabel('time')
ylabel('x_dot')
legend('actual','reference','Location','southeast')
title('x_dot')

subplot(2,2,2)
hold on
plot(time,xHistory(:,3))
plot(time,yreftot(:,3))
grid on
xlabel('time')
ylabel('psi')
legend('actual','reference','Location','southeast')
title('psi angle')

subplot(2,2,3)
hold on
plot(time,xHistory(:,5))
plot(time,yreftot(:,5))
grid on
xlabel('time')
ylabel('X')
legend('actual','reference','Location','southeast')
title('X position')

subplot(2,2,4)
hold on
plot(time,xHistory(:,6))
plot(time,yreftot(:,6))
grid on
xlabel('time')
ylabel('Y')
legend('actual','reference','Location','southeast')
title('Y position')

% 
% Plot the manipulated variables.
figure('Name','Control Inputs')

subplot(2,1,1)
hold on
stairs(time,uHistory(:,1))
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
grid on
xlabel('time')
legend('actual')
title('Input 1')

subplot(2,1,2)
hold on
stairs(time,uHistory(:,2))
% plot(time,nloptions.MVTarget(2)*ones(1,length(time)))
grid on
xlabel('time')
title('Input 2')
legend('actual')
% 

figure,plot(xHistory(:,5),xHistory(:,6));hold on;
plot(yreftot(:,5),yreftot(:,6));
