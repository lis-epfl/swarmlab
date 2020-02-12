function [V_command, collisions] = compute_vel_olfati_saber(self, p_swarm, r_agent, dt)
    % OLFATI-SABER SWARMING ALGORITHM
    % This is an implementation of the Olfati Saber algorithm. It allows the coherent
    % navigation of a swarm of agents, by preventing collisions, aligning their
    % velocities, avoiding obstacles and walls.
    %
    % Ref:
    % Inputs:
    %           p_swarm           swarming parameters
    %           r_agent     safety radius of agents
    %           dt          time step
    % Outputs:
    %           V_command   commanded velocities for every agent
    %           collisions  [nb_agent_collisions nb_obs_collisions min_dist_obs]
    %
    
    % Weight function for computing the motion planning acceleration
    psi = @(z) ((p_swarm.a + p_swarm.b) * (sqrt(1 + (z - p_swarm.d_ref + p_swarm.c)^2) - sqrt(1 + p_swarm.c^2)) + ...
        (p_swarm.a - p_swarm.b) * (z - p_swarm.d_ref)) / 2;
    psi_der = @(z) (p_swarm.a + p_swarm.b) / 2 * (z - p_swarm.d_ref + p_swarm.c) ./ sqrt(1 + (z - p_swarm.d_ref + p_swarm.c)^2) + ...
        (p_swarm.a - p_swarm.b) / 2;

    % Generalization of adjacency coefficients for computing the motion planning acceleration
    rho = @rho_f;
    rho_der = @rho_der_f;

    % Force defining the attraction/repulsion as function of the distance
    phi = @(z) 1 / p_swarm.r * rho_der(z, p_swarm.delta, p_swarm.r, p_swarm.k) * psi(z) + ...
        rho(z, p_swarm.delta, p_swarm.r, p_swarm.k) * psi_der(z);

    X = self.get_pos_ned();
    X = [X(1:2, :); -X(3, :)];
    V = self.get_vel_ned();
    V = [V(1:2, :); -V(3, :)];

    % Initialize variables
    nb_agents = self.nb_agents;
    M = zeros(nb_agents, nb_agents); % Neighborhood matrix
    D = zeros(nb_agents, nb_agents); % Distance matrix
    a_pm = zeros(3, nb_agents); % Position matching acceleration
    a_vm = zeros(3, nb_agents); % Velocity matching acceleration
    a_wall = zeros(3, nb_agents); % Arena repulsion acceleration
    a_obs = zeros(3, nb_agents); % Obstacle repulsion acceleration
    a_command = zeros(3, nb_agents); % Calculate the commanded acceleration
    V_command = zeros(3, nb_agents); % Calculate the commanded acceleration

    nb_agent_collisions = 0; % Nb of collisions among agents
    nb_obs_collisions = 0; % Nb of collisions against obstacles
    min_dist_obs = 20;

    for agent = 1:nb_agents
        p_rel = X - X(:, agent);
        dist = sqrt(sum((p_rel.^2), 1));
        D(agent, :) = dist;

        % Define neighbors list
        neig_idx = (1:nb_agents)';
        neig_idx = neig_idx(dist ~= 0);

        % Count collisions
        nb_agent_collisions = nb_agent_collisions + sum(dist < 2 * r_agent) - 1;

        % Number of neighbours
        nb_neig = nb_agents - 1;

        % Constraint on neighborhood given by the limited radius of
        % communication
        if isfield(p_swarm, 'r')
            neig_idx = neig_idx(dist(neig_idx) < p_swarm.r);
            nb_neig = length(neig_idx);
        end

        % Constraint on neighborhood given by the topological distance
        if isfield(p_swarm, 'max_neig')

            if nb_neig > p_swarm.max_neig
                [~, idx] = sort(dist(neig_idx));
                neig_idx = neig_idx(idx(1:p_swarm.max_neig));
                nb_neig = p_swarm.max_neig;
            end

        end

        % Adjacency matrix (asymmetric in case of limited fov)
        M(agent, neig_idx) = 1;

        % Compute acceleration vector for position and velocity matching
        if nb_neig ~= 0
            v_rel = V - V(:, agent);
            % v_rel_norm  = sqrt(sum((v_rel.^2),1));

            %Compute vel and pos unit vector between two agents
            p_rel_u = p_rel ./ dist;

            for agent2 = neig_idx'
                % Position matching
                a_pm(:, agent) = a_pm(:, agent) + phi(dist(agent2)) * p_rel_u(:, agent2);

                % Velocity alignement
                if ~p_swarm.is_active_migration
                    a_vm(:, agent) = a_vm(:, agent) + p_swarm.c_vm * v_rel(agent2);
                end

            end

        end

        % Add migration effect on accelerations
        if p_swarm.is_active_migration
            a_vm(:, agent) = p_swarm.c_vm * (p_swarm.u_ref * p_swarm.v_ref - V(:, agent));
        end

        % Add arena repulsion effect on accelerations
        if p_swarm.is_active_arena
            a_wall(:, agent) = repulsion_cubic_arena(X(:, agent), p_swarm.x_arena, p_swarm.d_arena, p_swarm.c_arena);
        end

        % Compute spheric effect
        if p_swarm.is_active_spheres

            for obs = 1:p_swarm.n_spheres
                % Get obstacle center and radius
                c_obs = p_swarm.spheres(1:3, obs);
                r_obs = p_swarm.spheres(4, obs);

                x_alpha = X(:, agent);
                v_alpha = V(:, agent);

                % Compute distance agent(a)-obstacle(b)
                dist_ab = sqrt(sum((X(:, agent) - c_obs).^2)) - r_obs;
                nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

                % Set the virtual speed of the obstacle direction out of
                % the obstacle
                % v_obs_virtual = (X(:,agent) - c_obs) / (dist_ab + r_obs) * S.v_shill;

                if dist_ab < min_dist_obs
                    min_dist_obs = dist_ab;
                end

                if (dist_ab < p_swarm.r0)
                    % Parameter s in [0,1]
                    s = r_obs / (r_obs + dist_ab);
                    x_beta = s * x_alpha + (1 - s) * c_obs;

                    % Derivative of s
                    s_dot = r_obs * (v_alpha' * (x_beta - x_alpha) / dist_ab) / (r_obs + dist_ab)^2;
                    v_beta = s * v_alpha - r_obs * (s_dot / s) * (x_beta - x_alpha) / dist_ab;
                    x_gamma = c_obs + p_swarm.lambda * p_swarm.u_ref;
                    d_ag = norm(x_gamma - X(:, agent));

                    % Acceleration effect
                    a_obs(:, agent) = a_obs(:, agent) + ...
                        +p_swarm.c_pm_obs * rho(dist_ab / p_swarm.r0, p_swarm.delta, p_swarm.r, p_swarm.k) * ...
                        (phi(dist_ab - p_swarm.d_ref) * (x_beta - x_alpha) / dist_ab + ...
                        phi(d_ag - p_swarm.d_ref) * (x_gamma - x_alpha) / (norm(x_gamma - x_alpha))) + ...
                        p_swarm.c_vm_obs * (v_beta - v_alpha);
                end

            end

        end

        % Compute cylindric effect
        if p_swarm.is_active_cyl

            for obs = 1:p_swarm.n_cyl
                % Get obstacle center and radius
                c_obs = p_swarm.cylinders(1:2, obs);
                r_obs = p_swarm.cylinders(3, obs);

                x_alpha = X(1:2, agent);
                v_alpha = V(1:2, agent);

                % Compute distance agent(a)-obstacle(b)
                dist_ab = sqrt(sum((X(1:2, agent) - c_obs).^2)) - r_obs;
                nb_obs_collisions = nb_obs_collisions + sum(dist_ab < r_agent);

                if dist_ab < min_dist_obs
                    min_dist_obs = dist_ab;
                end

                % Compute interaction effect
                if (dist_ab < p_swarm.r0)
                    % Parameter s in [0,1]
                    s = r_obs / (r_obs + dist_ab);
                    x_beta = s * x_alpha + (1 - s) * c_obs;

                    % Derivative of s
                    s_dot = r_obs * (v_alpha' * (x_beta - x_alpha) / dist_ab) / (r_obs + dist_ab)^2;
                    v_beta = s * v_alpha - r_obs * (s_dot / s) * (x_beta - x_alpha) / dist_ab;
                    x_gamma = c_obs + p_swarm.lambda * p_swarm.u_ref(1:2);
                    d_ag = norm(x_gamma - X(1:2, agent));

                    % Acceleration effect of the spheric obstacles
                    a_obs(1:2, agent) = a_obs(1:2, agent) + ...
                        +p_swarm.c_pm_obs * rho(dist_ab / p_swarm.r0, p_swarm.delta, p_swarm.r, p_swarm.k) * ...
                        (phi(dist_ab - p_swarm.d_ref) * (x_beta - x_alpha) / dist_ab + ...
                        phi(d_ag - p_swarm.d_ref) * (x_gamma - x_alpha) / (norm(x_gamma - x_alpha))) + ...
                        p_swarm.c_vm_obs * (v_beta - v_alpha);
                end

            end

        end

        a_command(:, agent) = a_pm(:, agent) + a_vm(:, agent) + ...
            +a_wall(:, agent) + a_obs(:, agent);

        % Integrate acceleration to get velocity
        V_command(:, agent) = V(:, agent) + a_command(:, agent) * dt;

    end

    % Total number of collisions per time step
    nb_agent_collisions = nb_agent_collisions / 2; % reciprocal
    collisions = [nb_agent_collisions nb_obs_collisions min_dist_obs];

    % Add random effect on velocities
    if isfield(p_swarm, 'c_r')
        V_command = V_command + p_swarm.c_r * randn(3, nb_agents);
    end

    % Bound velocities
    if isempty(p_swarm.max_v)
        v_norm = sqrt(sum((V_command.^2), 1));
        idx_to_bound = (v_norm > p_swarm.max_v);

        if sum(idx_to_bound) > 0
            V_command(:, idx_to_bound) = p_swarm.max_v * ...
                V_command(:, idx_to_bound) ./ repmat(v_norm(idx_to_bound), 3, 1);
        end
    end
    if isempty(p_swarm.max_a)
        accel_cmd = (V_command-V)./dt;
        accel_cmd_norm = sqrt(sum(accel_cmd.^2, 1));
        idx_to_bound = ( accel_cmd_norm > p_swarm.max_a | accel_cmd_norm < - p_swarm.max_a);
        if sum(idx_to_bound) > 0
            V_command(:, idx_to_bound) = V(:, idx_to_bound) + ...
                dt*p_swarm.max_a * accel_cmd(:, idx_to_bound) ./ ...
                repmat(accel_cmd_norm(idx_to_bound), 3, 1);
        end

    end

    V_command = [V_command(1:2, :); -V_command(3, :)];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RHO - It is the function defining the adjacency coefficients
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = rho_f(z, delta, r, k)

    if z < delta * r
        y = 1;
        return;
    elseif z < r
        y = (1/2^k) * (1 + cos(pi * (z / r - delta) ./ (1 - delta)))^k;
        return;
    else
        y = 0;
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Derivative of rho
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = rho_der_f(z, delta, r, k)

    if z < delta * r
        y = 0;
        return;
    elseif z < r
        arg = pi * (z / r - delta) ./ (1 - delta);
        y = -pi / (1 - delta) * k / (2^k) * (1 + cos(arg))^(k - 1) * (sin(arg));
        return;
    else
        y = 0;
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Repulsion from cubic arena
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function a_arena = repulsion_cubic_arena(pos_drone, pos_walls, wall_width, const_rep)

    a_arena = zeros(3, 1);

    for axis = 1:3
        x_drone = pos_drone(axis);
        % Repulsion from left wall
        x_arena = pos_walls(axis, 1);

        if x_drone < x_arena
            a_arena(axis) = a_arena(axis) + const_rep;
        elseif x_drone > x_arena && x_drone < x_arena + wall_width
            a_arena(axis) = a_arena(axis) + const_rep * 0.5 * (sin((pi / wall_width) * (x_drone + x_arena) + pi / 2) + 1);
        end

        % Repulsion from right wall
        x_arena = pos_walls(axis, 2);

        if x_drone > x_arena - wall_width && x_drone < x_arena
            a_arena(axis) = a_arena(axis) - const_rep * 0.5 * (sin((pi / wall_width) * (x_drone - x_arena - wall_width) - pi / 2) + 1);
        elseif x_drone > x_arena
            a_arena(axis) = a_arena(axis) - const_rep;
        end

    end

end
