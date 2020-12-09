function [safety, order, union, alg_conn, safety_obs, min_d_obs ] = ...
    compute_swarm_performance(pos_history, vel_history, ...
    p_swarm, dirname)
% Compute swarm performance - This function allows to compute the 
% performance from the history of the swarming variables (time, position, 
% velocity, acceleration).
%
% Inputs:
%   pos_history: series of agents' positions
%   vel_history: series of agents' velocities
%   p_swarm: structure with swarm parameters
%   dirname: path of the directory where results are saved
%
% Outputs:
%   safety: safety metric. It reflects the number of agent-agent collisions
%   order: order metric. It measures the correlation between velocity
%               vectors
%   union: union metric. It measures the number of connected components of
%               the undirected graph
%   alg_conn: algebraic connectivity metric.
%   safety_obs: safety againt obstacles metric. reflects the number of 
%               agent-obstacle collisions
%   min_d_obs: minimum distance agent-obstacle
%

%% Init variables

[t_steps,nx] = size(pos_history);
nb_agents = nx/3;
M = zeros(t_steps,nb_agents,nb_agents);
nb_ag_coll = zeros(t_steps,1);
nb_obs_coll = zeros(t_steps,1);
nb_conn_comp = zeros(t_steps,1);

safety = ones(t_steps,1);
order = zeros(t_steps,1);
union = zeros(t_steps,1);
alg_conn = zeros(t_steps,1);
safety_obs = zeros(t_steps,1);
min_d_obs = zeros(t_steps,nb_agents);

%% Loop over time

for k = 1:t_steps
    
    %% Safety: reflects the number of collisions among the swarm agents
    
    pos_k = pos_history(k,:);
    pos_k = reshape(pos_k,3,[]);
    dist_vect_k = pdist(pos_k');
    if ~isempty(dist_vect_k)
        nb_ag_coll(k) = sum(dist_vect_k < 2*p_swarm.r_coll);
        safety(k) = 1 - (sum(nb_ag_coll(k)) / length(dist_vect_k));
    else
        nb_ag_coll(k)=0;
        safety(k) = 1;
    end
    
    %% Order: reflects the correlation of the velocity vectors
    distance_matrix_k = squareform(dist_vect_k);
    M(k,:,:) =  compute_neighborhood(distance_matrix_k, p_swarm.r, p_swarm.max_neig);
    for agent = 1:nb_agents
        neig = (M(k,:,agent)==true);
        nn = sum(neig);
        vel_k = vel_history(k,:);
        vel_k = reshape(vel_k,3,[]);
        if nn ~= 0
            scalar_prod = vel_k(:, agent)' * vel_k(:, neig);
            speed_agent = norm(vel_k(:, agent));
            speed_neig = sqrt(sum((vel_k(:, neig) .^ 2), 1));
            order(k) = order(k) + ...
                sum (scalar_prod ./ (speed_agent * speed_neig)) / nn;
        else
            order(k) = order(k) + 1;
        end
    end
    order(k) = order(k)/nb_agents;
    
    %% Union: reflects the number of connected components in the related
    % symeetric graph
    
    M_k = squeeze(M(k,:,:));
    A = ((M_k + M_k') > 0);
    [nb_conn_comp(k), ~, ~]  = network_components(A);
    union(k) = (nb_agents - nb_conn_comp(k))/(nb_agents-1);
    
    %% Connectivity: reflects the algebraic connectivity in the related
    % symmetric graph
    
    alg_conn(k) = compute_alg_connectivity(A)/nb_agents;
    
    %% Safety with obstacles: reflects the number of collisions between swarm agents and obstacles
    
    nb_possible_coll = 0;
    if p_swarm.is_active_spheres | p_swarm.is_active_cyl | p_swarm.is_active_arena
        
        pos_k = pos_history(k,:);
        pos_k = reshape(pos_k,3,[]);
        
        if p_swarm.is_active_spheres
            c_spheres = p_swarm.spheres(1:3,:);
            r_spheres = p_swarm.spheres(4,:);
            
            D_spheres = pdist2(pos_k',c_spheres');
            min_d_obs(k,:) = min(pdist2(pos_k(1:2,:)',c_spheres') - repmat(r_spheres, nb_agents, 1),[],2); 
            nb_obs_coll(k) = nb_obs_coll(k) + sum(sum(D_spheres < repmat(r_spheres, nb_agents, 1)));
            nb_possible_coll = nb_possible_coll + nb_agents*length(r_spheres);
        end
        if p_swarm.is_active_cyl
            c_cyl = p_swarm.cylinders(1:2,:);
            r_cyl = p_swarm.cylinders(3,:);
            
            D_cyl = pdist2(pos_k(1:2,:)',c_cyl');
            min_d_obs(k,:) = min(pdist2(pos_k(1:2,:)',c_cyl') - repmat(r_cyl, nb_agents, 1),[],2); 
            nb_obs_coll(k) = nb_obs_coll(k) + sum(sum(D_cyl < repmat(r_cyl, nb_agents, 1)));
            nb_possible_coll = nb_possible_coll + nb_agents*length(r_cyl);
        end
        if p_swarm.is_active_arena
            x_coll = sum(pos_k(1,:)< p_swarm.x_arena(1,1) | pos_k(1,:)> p_swarm.x_arena(1,2));
            y_coll = sum(pos_k(2,:)< p_swarm.x_arena(2,1) | pos_k(2,:)> p_swarm.x_arena(2,2));
            z_coll = sum(pos_k(3,:)< p_swarm.x_arena(3,1) | pos_k(3,:)> p_swarm.x_arena(3,2));
            nb_obs_coll(k) = nb_obs_coll(k) + x_coll + y_coll + z_coll;
            nb_possible_coll = nb_possible_coll + nb_agents;
        end
        
        safety_obs(k) = 1 - (nb_obs_coll(k)/nb_possible_coll);
    end
    
end

%% Save workspace

if ~isempty(dirname)
    path = strcat(dirname,'/performance');
    save(path);
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function connectivity = compute_alg_connectivity(A)
% Compute alg connectivity - compute algebraic connectivity of the graph
% associated to the swarm at a given time step.

D = diag(sum(A, 2)); % degree matrix
L = D - A;           % laplacian matrix
eigenvalues = eig(L);
eigenvalues = sort(eigenvalues);
connectivity = eigenvalues(2);

end
