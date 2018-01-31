function [A,B,K]=SSsystem()
initparams;initequil;
global K
    global g m IB Izzp  IT   l Dt Kf Kt 
    
    global  wbar fbar wbbar nbar
    
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
end