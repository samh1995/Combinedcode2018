function []= initparams()
    global g m IB  Izzp  IT damping_ratio  nat_freq
    
    global   l Dt Kf Kt 
% %% Navi
    g = 9.81;   % gravity
    m =  1.095;  % mass (kg) 
    
    % inertial properties of all vehicle
 
    
    % inertial properties of one propeller
    Izzp=2.20751e-5;
    % inertial properties in body frame
Ixx = 0.01121976;
Iyy = 0.01122668;
Izz = 0.021082335;
Ixy = -5.62297e-05;
Iyz = -4.4954e-06;
Izx = -1.418e-08;  
  
    IzzT=Izz +Izzp;
    
    IB = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 
    %Ip = [Ixxp Ixy Izx;Ixy Iyyp Iyz;Izx Iyz Izzp]; 
    IT = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz IzzT]; 
   
    l=0.1807; %distance of prop to cog
    
  
    Kf = 7.933*10^-6; % Thrust coefficient
    Kt = 0.011045977; % torq coefficient
    Dt= 5*10^-3; % drag coeffient
% Dt=0; %its like taht in navi
    damping_ratio=0.7;
    nat_freq=1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555555
    %% Dandrea
%        g = 9.81;   % gravity
%     m = 0.5;  % mass (kg) 
%     
%     % inertial properties of all vehicle
%     IxxT=0.0027;
%     IyyT=0.0027;
%     IzzT=0.005215;
%     
%     % inertial properties of one propeller
%     Izzp=1.5*10^-5;
%     % inertial properties in body frame
%     Ixx = 0.0027;
%     Iyy = 0.0027;
%     Izz = 0.0052;
%     
%     Ixy = 0;
%     Iyz = 0;
%     Izx = 0;
%     
%     
%     IB = [Ixx Ixy Izx;Ixy Iyy Iyz;Izx Iyz Izz]; 
%     %Ip = [Ixxp Ixy Izx;Ixy Iyyp Iyz;Izx Iyz Izzp]; 
%     IT = [IxxT Ixy Izx;Ixy IyyT Iyz;Izx Iyz IzzT]; 
%    
%     l=0.17; %distance of prop to cog
%     Kf = 6.41*10^-6; % Thrust coefficient
%     Kt = 1.69*10^-2; % torq coefficient
%     Dt= 2.75*10^-3; % drag coeffient
% 
%     damping_ratio=0.7;
%     nat_freq=1;
%     
%     
    
    
end

