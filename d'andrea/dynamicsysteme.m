function [stateDeriv]= dynamicsysteme(t, state, f,desirestate)
% Propagates quadrotor dyanimcs using ode45
% Inputs: time, state and control signal 
% Output: derivative of the state


  % new state (u,v,w,p,q,r,X,Y,Z,q0,q1,q2,q3) uvw in body fram
global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq  
global  wbar fbar wbbar nbar K

IP=[0 0 0; 0 0 0;0 0 Izzp];
q = [state(10);state(11);state(12);state(13)]/norm(state(10:13));
R = quat2rotmat(q); %% from body to inertial
state = reshape(state,[max(size(state)),1]);
stateDeriv = zeros(13,1);

%making sure the forces for the 4 propellers are acceptable 
j=1;
for j=1:3
%     if f(j)>3.8
%        f(j)=3.8;
%     end
%    if f(j)<0.2
%        f(j)=0.2;
%    end
   w(j)=sqrt(abs(f(j))/Kf) ;
end
J=[0;0;sum(w)];
w(4)=0;
f(4)=0;

%%rpmDeriv = (rpm2rad(rpm) - rpm2rad(rpmPrev))/tStep; %in rad/s^2%im in secs?minute? or secs?% need to
%%add this later on!!
fThrust = [0; 0; Kf*sum(w.^2)]; %express fthrust in body frame

fGravity = inv(R)*[0; 0; -m*g];   %change fgravity to body frame 

M=[ (f(2)-f(4))*l;  (f(3)-f(1))*l;Kt*(f(1)-f(2)+f(3)-f(4))-Dt*state(6)];


stateDeriv(1:3) = (fGravity  +fThrust - m*cross(state(4:6),state(1:3)))/m;
stateDeriv(4:6) = inv(IB)*(M-cross(state(4:6),IB*state(4:6)+IP*(state(4:6)+J)));
stateDeriv(7:9)=R*state(1:3);
stateDeriv(10:13) = 0.5*quatmultiply(q,[0;state(4:6)]);

% stateDeriv(4:6) = inv(I)*(M-cross(state(4:6),I*state(4:6)+IP*(state(4:6)+sum(W(1)+W(2)+W(3)+W(4)))));
% stateDeriv(4)=(1/IB(1,1))*((f(2))*l-state(5)*state(6)*(IT(3,3)-IT(1,1))-Izzp*state(5)*(sqrt(abs(f(1))/Kf) +sqrt(abs(f(2))/Kf) +sqrt(abs(f(3))/Kf)+ sqrt(abs(f(4))/Kf) ));
% stateDeriv(5)=(1/IB(1,1))*((f(3)-f(1))*l+state(4)*state(6)*(IT(3,3)-IT(1,1))+Izzp*state(4)*(sqrt(abs(f(1))/Kf) +sqrt(abs(f(2))/Kf) +sqrt(abs(f(3))/Kf)+ sqrt(abs(f(4))/Kf)));
% stateDeriv(6)=(1/IB(3,3))*(-Dt*state(6)+Kt*(f(1)-f(2)+f(3)-f(4)));

end
