function [Hist] = inithist(state,t,f,desiredstate)
    % Initialize history of the state and its derivative
    Hist.states = state;
    %Hist.stateDerivs = stateDeriv;
    Hist.times = t;
 Hist.f = f;

 Hist.desiredstate=desiredstate;
 
    % Initialize history of twist, pose and control structs
   % Hist.u = u;
    
end

%test