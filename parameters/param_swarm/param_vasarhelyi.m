%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Vasarhelyi Paramteres
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Repulsion

% Repulsion range
p_swarm.r0_rep = p_swarm.d_ref; % radius of repulsion
% Repulsion gain
p_swarm.p_rep = 0.03;


%% Friction

% Stopping point offset of alignment
p_swarm.r0_fric = 85.3;
% Coefficient of velocity alignment
p_swarm.C_fric = 0.05;
% Velocity slack of alignement
p_swarm.v_fric = 0.63;
% Gain of braking curve
p_swarm.p_fric = 3.2;
% Acceleration of braking curve
p_swarm.a_fric = 4.16;


%% Obstacles and wall parameters

% Stopping point offset of walls
p_swarm.r0_shill = 0.3;
% Velocity of virtual shill agents
p_swarm.v_shill = 13.6;
% Gain of bracking curve for walls
p_swarm.p_shill = 3.55;
% Acceleration of braking curve for walls
p_swarm.a_shill = 3.02;    
