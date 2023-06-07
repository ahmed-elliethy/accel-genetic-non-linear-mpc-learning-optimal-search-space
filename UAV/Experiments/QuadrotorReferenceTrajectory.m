function [ xdesired ] = QuadrotorReferenceTrajectory( t , kk )
% This function generates reference signal for nonlinear MPC controller
% used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

%#codegen
x =  6*sin(t*(kk));
y = -6*sin(t*(kk)).*cos(t*(kk));
z = 6*cos(t*(kk));
% z =  6*t/3 ;
% x = 6*t/3 ;
% for k =1:length(t)
%  x(k) =  3;
%     if (t(k) >= 30 && t(k)< 70)
%         x(k) = 7 ;
%     elseif (t(k)>=70 && t(k)< 100)
%         x(k) = t(k)/10;
%      elseif (t(k)>=100 && t(k)< 150)
%         x(k) = 15;
%      elseif (t(k)>=150)
%         x(k) = t(k)/10;
%  end
% end

% y =  6*t/3 ;
% Ts =.1;
% z = [6*cos(t(1:10/Ts)/3) 6*cos(t(10/Ts+1:end)/3)];

phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));

xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end

