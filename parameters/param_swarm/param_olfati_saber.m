%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Olfati Saber parameters
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Coefficients for velocity matching

p_swarm.c_vm  = 3;

p_swarm.a     = 1;
p_swarm.b     = 5;
p_swarm.c     = (p_swarm.b-p_swarm.a)/(2*sqrt(p_swarm.a*p_swarm.b));
p_swarm.delta = 0.2;
p_swarm.k     = 2;

% Velocity of migration - replace the velocity matching and it uses the
% same gain P.c_cm
% P.v_migration = [0 4 0]';  

%% Obstacle parameters

p_swarm.r0 = 10;
p_swarm.lambda = 1;               % (0,1]
p_swarm.c_pm_obs = 5;
p_swarm.c_vm_obs = p_swarm.c_vm;
