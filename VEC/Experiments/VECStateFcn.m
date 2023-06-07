function dx = VECStateFcn(states, U)
    % Get the necessary constants    
    g=9.81;
    m=1500;
    Iz=3000;
    Cf=38000;
    Cr=66000;    
    rho=1.225;
    lf=2;
    lr=3;
    Ts=0.02;
    mju=0.02;
    
    x_dot=states(1);
    y_dot=states(2);
    psi=states(3);
    psi_dot=states(4);
    X=states(5);
    Y=states(6);
    
    % Inputs:
    delta  = U(1);
    a  = U(2);
    
    Fyf=Cf*(delta-y_dot/x_dot-lf*psi_dot/x_dot);
    Fyr=Cr*(-y_dot/x_dot+lr*psi_dot/x_dot);
   
    % The nonlinear equation describing the dynamics of the drone
    dx(1,1)=a+(-Fyf*sin(delta)-mju*m*g)/m+psi_dot*y_dot;
    dx(2,1)=(Fyf*cos(delta)+Fyr)/m-psi_dot*x_dot;
    dx(3,1)=psi_dot;
    dx(4,1)=(Fyf*lf*cos(delta)-Fyr*lr)/Iz;
    dx(5,1)=x_dot*cos(psi)-y_dot*sin(psi);
    dx(6,1)=x_dot*sin(psi)+y_dot*cos(psi);
    
end