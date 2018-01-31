function [Control] = controllerfailrecovery(tStep, Pose, Twist, Control)
% Performs collision recovery control.
%
% Inputs: 
%   dt              :   time step (default 1/200 sec)
%   Pose            :   Struct of the pose of the quadrotor
%   Twist           :   Struct of the twist of the quadrotor
%   Control         :   Struct of the control variables and output
%   Hist.control    :   History of control signals
%
% Outputs:
%   Control         :   Struct of the control variables and output
global m Ixx Iyy Izz u2RpmMat Izzp IzzT IT
global g m IB Izzp IT l Dt Kf Kt  damping_ratio nat_freq 
global  wbar fbar wbbar nbar
%% Rotation from Fiona to dandrea coordinates
R_F2D=[ sqrt(2)/2    sqrt(2)/2            0;
        sqrt(2)/2     -sqrt(2)/2          0;
        0               0                -1];
quat = [Pose.attQuat(1);Pose.attQuat(2);Pose.attQuat(3);Pose.attQuat(4)]/norm(Pose.attQuat(1:4));
R_I2F = quat2rotmat(quat); %from inertial to fiona
R_I2D=R_F2D*R_I2F;
%% LQR controller: solving for k 
a=(IT(1,1)-IT(3,3))*wbbar(3)/IB(1,1)-Izzp*(wbar(1)+wbar(2)+wbar(3))/IB(1,1);

A=[0    a    0    0 ;
   -a   0    0    0 ;
   0  -nbar(3) 0 wbbar(3);
   nbar(3) 0 -wbbar(3) 0;];
B=l/IB(1,1)*[0 1;1 0; 0 0; 0 0];
C=[0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0];
D=[0 0;0 0 ;0 0 ;0 0 ];
q=[1 1 20 20];
Q=diag(q);
R=[ 1 0; 0 1];
Y=0;
N=0;                   
[K,S,e]=lqr(A,B,Q,R,N);

%% Solving for control inputs
% % Euler=R_F2D*Pose.attEuler(1:3);
% % q_dand = angle2quat(Euler(3),Euler(2),Euler(1),'ZYX'); %find dand quat using the euler found from fionas quat
% % R_dand=quat2rotmat(q_dand);%% from dandrea body to inertial frame
f_total=norm(m/nbar(3)*(Control.desiredaccel-[0;0;-g])) %ftotal in world frame

n_desired=m/nbar(3)*R_I2D*(Control.desiredaccel-[0;0;-g])/norm(m/nbar(3)*(Control.desiredaccel-[0;0;-g]));
% n_desired=m/nbar(3)*quat2rotmat(Pose.attQuat)*R_F2D*(Control.desiredaccel-[0;0;-g])/norm(m/nbar(3)*(Control.desiredaccel-[0;0;-g]));%ndeisred in body frame. quat2rotmat converts from inertial to body it was invR
a=R_F2D*[Twist.angVel(1);Twist.angVel(2);Twist.angVel(3)];
s=[a(1)-wbbar(1);a(2)-wbbar(2);n_desired(1)-nbar(1);n_desired(2)-nbar(2)];
u=-K*s;

  
%% solving for forces convert to rpm
  G=[1 1 1; -1 0 1; 0 1 0];
  H=[(f_total); u(1)-fbar(1)+fbar(3);u(2)+fbar(2)];
  f=linsolve(G,H);
  f(4)=0;
  Control.rpm=sqrt(f./kt);
end