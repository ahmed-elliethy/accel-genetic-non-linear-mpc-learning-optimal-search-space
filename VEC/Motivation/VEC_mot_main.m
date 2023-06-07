% Control of VEC Using Nonlinear Model Predictive Control
clear all; close all;clc;
nx = 6;
ny = 6;
nu = 2;
nlobj = nlmpc(nx, ny, nu);
nlobj.Model.StateFcn = "VECStateFcn";
% nlobj.Jacobian.StateFcn = @VECStateJacobianFcn;
% rng(0)
% validateFcns(nlobj,rand(nx,1),rand(nu,1));
Duration = 40; 

Ts = .02;
p = 20;
c = 20; n=2;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;
nlobj.MV = struct( ...
    Min={-pi/6;-30}, ...
    Max={pi/6;30} ,...
    RateMin={-pi/30;-.1}, ...
    RateMax={pi/30;.1} ...
    );
nlobj.Weights.OutputVariables = [10000 0 10000 0 100 100];
q = [10000 10000 100 100];
nlobj.Weights.ManipulatedVariables = [100 1];
r = [100 1];
nlobj.Weights.ManipulatedVariablesRate = [100 1];
% Closed-Loop Simulation
% Specify the initial conditions
% Nominal control that keeps the VEC floating
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [0 0]; 
mv = nloptions.MVTarget;
kkk=1;
% Simulate the closed-loop system using the nlmpcmove function, specifying simulation options using an nlmpcmove object.
% OUTER LOOP

for kk=1:1
f =     (.01-(.001))*rand(1,1)+.001;
f= 0.01;
% loop1 = (16-(12))*rand(1,1)+12;
% loop2 = (12-(8))*rand(1,1)+8;
% loop3 = (2.1-(1.9))*rand(1,1)+1.9;
% amp = (.1-(.001))*rand(1,1)+.001;
% org = (500-(0))*rand(1,1)+(0);
% rad = (1000-(100))*rand(1,1)+(100);

t = 0:Ts:Duration;
yref = VECReferenceTrajectory(Ts,t,f,14,10,2,.1,0,100);
xHistory = yref(1,:);  
lastMV = mv; uHistory = lastMV;
uk_pred = mv; 
nloptions.MV0 = zeros(p,n);    
hbar = waitbar(0,'Simulation Progress'); 
% inner loop
 for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
yref = VECReferenceTrajectory(Ts,t,f,14,10,2,.1,0,100);
    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
% calculate the first control input of the current cycle based on fault states
    uk_pred = nloptions.MV0(2:end,:) ;
    rrr1= (.1-(-.1))*rand(1,2)+(-.1);  
    rrr2= (.001-(-.001))*rand(1,2)+(-.001);  
    xk1 =   [xk(1) xk(3) xk(5) xk(6)] ;
    xxk1 = xk1 + [rrr1 rrr2]; 
    xxk =  [xxk1(1) xk(2) xxk1(2) xk(4) xxk1(end-1:end)];
    Beta(kkk,:) =  max(calcFeature(xxk1, xk1, q)); 
    
    [uk,nloptions,info] = nlmpcmove(nlobj,xxk,lastMV,yref,[],nloptions);
% claculate BSM 
    ukk = nloptions.MV0(1:end-1,:) ;
    BSM(kkk,:) = max(max(abs(ukk - uk_pred)./[0.4188,.4]));  kkk=kkk+1;
%   Update states
    xk = getstates(xxk,uk,Ts);
    uHistory(k+1,:) = uk';
    duHistory(k,:) = abs(uHistory(k+1,:) - uHistory(k,:)) ;
    lastMV = uk;
    xHistory(k+1,:) = xk;
    waitbar(k*Ts/Duration,hbar);
 end
 close all;
 plotVECTrajectory;
 close(hbar);
 kk
end
a = [Beta,BSM];
figure,plot(a);legend('feature','BSM');
b = sortrows(a,1);
a2 = reshape(b(:,2),length(BSM)/10,[]);
figure,boxplot(a2);
str ='\Delta^{max}_c' ;
ylabel(str,'interpreter','tex','FontSize',20)
xlabel('Percentiles','FontSize',16)

function [Beta] =  calcFeature(xxk,xk,q)
    Beta = (xxk - xk).^2 .*q ;
end
