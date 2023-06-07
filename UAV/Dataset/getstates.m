function xkp1 = getstates(xk,uk,Ts)
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xk);
    xkp1 = YOUT(end,:);
end
