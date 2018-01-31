clear all;
clc;
initparams;
initequil;
    global g m I Ixx Iyy Izz Izzp Ixxp IT Ip  l Dt Kf Kt  damping_ratio nat_freq 
    
    global  wbar fbar wbbar nbar K
    state=[0 0 0 0 0 20 0 0 10 0 0 0]';
    f=[2.2 2.1 0.8 0];
    desiredstate=[0 0 0 0 0 0 5 5 10 0 0 0];
    fic=[2.2 2.1 0.8];
    endTime = 50;  % seconds
    dt = 1 / 50; % time step (Hz)

    %initialconditions=[iC(1);iC(2)-5.69;iC(3);iC(4)-0.289];
    %initialconditions=[11;2;0.95;0.3];

    Hist = inithist(state, stateDeriv,u,desiredstate);
    W=1;

for i = 0 : dt : endTime - dt
    [A,B,K]=SSsystem(); %finds K for LQR system
    errouter_d =[state(7)-desiredstate(7);state(8)-desiredstate(8);state(9)-desiredstate(9)];
    errouter_d_dot=[state(10)-desiredstate(10);state(11)-desiredstate(11);state(12)-desiredstate(12)];
    desired_accl=-2*
    options = odeset('RelTol',1e-3); %%tolerance
    [tODE,y] = ode45(@(tODE, y) sys( y, A,B,K,tODE, dt),[i i+dt], initialconditions, options);
    U=-K*y(end,:)';
    T = fsolve(@(x) forces(x,U), fic);
    f(W,:)=T;
    u(W,:)=U';
    W=W+1;
  
   %y(i,:)=y(i,:)-transpose(initialconditions); 
  
end

figure(1)
plot(t,y(:,3));
% %    options = odeset('RelTol',1e-3); %%tolerance
% %     [tODE,errODE] = ode45(@(tODE, errODE) eq38error( errODE, tODE, dt),[i i+dt], err', options);