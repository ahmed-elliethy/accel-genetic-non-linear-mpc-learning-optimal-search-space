% UAV Exp1
function [C1,E1,P1] = UAV_exp1_FG(Duration)
% getQuadrotorDynamicsAndJacobian;
p = 10; c=10; n = 4 ; Ts =.4 ;   mstates = 6; tt=0;ee=0;pp=0;
E0=zeros(1,mstates); C0=zeros(1,mstates); 
q =  [1 1 1]; 
uref = 4.9*ones(p,n);  r = [0.1 0.1 0.1 0.1];
lastMV = 4.9*ones(1,c*n); uHistory = lastMV;  kk=.3;
hbar = waitbar(0,'Simulation Progress');
    t = linspace(Ts,p*Ts,p);yref = QuadrotorReferenceTrajectory(t,kk);
     xHistory = yref(:,1)';
     
for k = 1:(Duration/Ts)-p
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t,kk);
    xk = xHistory(k,:); 
    rrr= (1-(-1))*rand(1,3)+(-1); 
    xk1 =   xk(1:3) ;
    xxk1 = xk1 + rrr; 
    xxk =  [xxk1(1:3) xk(4:12)];
    Beta =  calcFeature(xxk1, xk1, q);
%         BSM= [u1.predictFcn(Beta),u2.predictFcn(Beta),u3.predictFcn(Beta),u4.predictFcn(Beta)];
    BSM = 8*ones(1,4);
        tic
    [uk,cost,pop_size,convergence] = GAsolver2(BSM,xxk,lastMV,yref',p,c,n,Ts,uref);
    if convergence == true 
        pp = pp+1;
    end
    tt = tt + toc ;  ee = ee + cost; pop(k) = pop_size; BS(k,:) = BSM;
    uHistory(k+1,:) = uk;              lastMV = uk;
    xk = getstates(xxk,uk(1:n)',Ts);
    xHistory(k+1,:) = xk;
    waitbar(k*Ts/Duration,hbar);
end
close(hbar)
%plotQuadrotorTrajectory
C1 = tt/k ;
E1 = ee ;
P1 = pp/k*100;
end

function [Beta] =  calcFeature(xxk,xk,q)
    Beta = (xxk - xk).^2 .*q ;
end
