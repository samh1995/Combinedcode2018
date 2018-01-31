function [ds]=sys(s,A,B,K, t,dt)
initparams;initequil;
global K
    global g m I Ixx Iyy Izz Izzp Ixxp IT Ip  l Dt Kf Kt 
    
    global  wbar fbar wbbar nbar
    

ds=A*s-B*K*s;
end