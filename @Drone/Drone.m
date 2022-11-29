classdef Drone < handle
    % DRONE : This class is meant to create and manage drones.
    % Every drone is either a fixed-wing or a quadcopter (drone_type).
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Drone general properties:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % drone_type:
    %           enumerator class DroneType
    % autopilot_version:
    %           1 for quad attitude, 2 for quad speed, 3 for quad acc
    % p_drone, p_battery, p_sim, p_physics, map:
    %           parameter structures associated to the drone (p_drone),
    %           the battery (p_battery), the simulation (p_sim), physics (p_physics) or the map (map)
    % pos_ned:
    %           vect3, position coordinates in the inertial frame
    %           (North, East and Down) [m]
    % vel_xyz:
    %           vect3, velocity in body frame (vx, vy, vz) [m/s]
    % attitude:
    %           vect3, attitude of the drone (phi, theta, psi) [rad]
    % rates:
    %           vect3, angular rates w.r.t. phi, theta and psi  [rad/s]
    % pos_ned_history: matrix of size n, 3: each row is vect of previous
    %                  pos_ned, position coordinates in the inertial frame 
    %                  (North, East and Down) [m]
    %
    % vel_xyz_history: matrix of size n, 3: each row is vect of previous
    %                  vel_xyz, velocities in the body frame 
    %                  (North, East and Down) [m/s]
    % prev_state:
    %           vect12, contains the previous state.
    %           state = (pos_ned, vel_xyz, attitude, rates)
    % path_len:
    %           path lenght of the drone, from the beginning of the
    %           simulation
    % z_hat:
    %           vect22, estimated extended state
    % airdata:
    %           vect6, (va, alpha, beta, wn, we, wd)
    % command:
    %           vect4, commmand input. The variables contained depend on
    %           the autopilot version
    % prev_command:
    %           vect4, store the previous command input
    % full_command:
    %           vect19, full command state vector, used in the function
    %           plot_uav_state_variable
    % delta:
    %           vect4,
    %           for quadcopter: normalized angular velocities commanded to
    %           the 4 motors
    % forces:
    %           vect3, 3D vector of the forces acting on the drone [N]
    %           computed in the body frame
    % moments:
    %           vect3, 3D vector of the moments acting on the drone [N*m]
    %           computed in the body frame
    % wind:
    %           vect6, wind and gusts in xyz coordinates
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Handles and other variables for the autopilot:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % body_handle:
    %           graphic handle associated to the body of the drone
    % path_handle:
    %           graphic handle associated to the path of the drone
    % waypoint_handle:
    %           graphic handle associated to the waypoints
    % vertices:
    %           
    % faces:
    %           drone (sur)faces
    % face_colors:
    %           colors associated to the drone surfaces in the plot
    % color:
    %           color associated to the drone for scatterplots, offline
    %           trajectory plots, ecc.
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Variables for the battery:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % i:
    %           vect5, current intensity over a 5-time-step window [A]
    % Q:
    %           capacity consumed from the beginning of the flight [Ah]
    % Qt:
    %           capacity consumed in a time step dt [Ah]
    % V:
    %           voltage [V]
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Variables for path planner/manager:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % path:
    %           vect26 // TODO: describe the content
    % nb_waypoints:
    %           nb of waypoints that the drone follow
    % waypoints:
    %           vect5*nb_waypoints to reshape before using
    % transition_time:
    %           time at which the transition to the new waypoint started
    % alpha:
    %           1 when transition starts, 0 at the end
    % command_before_transition: 
    %           // TODO: describe the content
    %

    properties

        % Parameters
        drone_type DroneType
        autopilot_version
        p_drone
        p_battery
        p_sim
        p_physics
        
        map % FIXME: why does the drone need a map? That's not logical

        % State = [pos_ned; vel_xyz; attitude; rates]
        pos_ned     % pn, pe, pd
        vel_xyz     % vx, vy, vz
        attitude    % phi, theta, psi
        rates       % p, q, r
        
        % State history
        pos_ned_history
        vel_xyz_history

        % Auxiliary variables
        prev_state
        path_len
        y       % sensor measurements
        z_hat   % esitmated extended state
        airdata % air data

        % Command variables
        command
        prev_command
        full_command
        delta

        % Forces and moments
        forces
        moments
        wind

        % Set handles and other variables for plots
        body_handle
        path_handle
        waypoint_handle
        vertices
        faces
        face_colors
        color
        state_handle

        % Variables for autopilot-quad
        altitude_state
        altitude_state_prev
        initialize_integrator
        P_h_thrust
        P_roll_torque
        P_pitch_torque
        P_psi_torque
        P_ad_thrust
        P_vn_pitch
        P_ve_roll
        P_vd_thrust
        P_ae_roll
        P_an_pitch
        P_pd_thrust
        P_pe_roll
        P_pn_pitch

        % Variables for autopilot-wing
        lat_state
        lat_state_prev
        lat_init_integrator
        lat_counter_last_change
        P_phi_da
        P_chi_phi
        P_theta_de
        P_h_theta
        P_va_dt
        P_va_theta

        % Variables for the battery
        i
        Q
        Qt
        V

        % Variables for path planner/manager
        path
        nb_waypoints
        waypoints
        transition_time
        alpha
        command_before_transition
        
        use_estimation = false

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods

        function Drone = Drone(drone_type, ...
                p_drone, p_battery, p_sim, p_physics, map)

            % Create a drone: assign parameters and initialize state to 0
            Drone.drone_type = drone_type;
            
            switch drone_type
                case "quadcopter"
                    Drone.autopilot_version = 2;
                case "fixed_wing"
                    Drone.autopilot_version = -1;
                case "point_mass"
                    Drone.autopilot_version = 2;
            end
            
            Drone.p_drone = p_drone;
            Drone.p_battery = p_battery;
            Drone.p_sim = p_sim;
            Drone.p_physics = p_physics;
            Drone.map = map;

            Drone.pos_ned = zeros(3, 1);
            Drone.vel_xyz = zeros(3, 1);
            Drone.attitude = zeros(3, 1);
            Drone.rates = zeros(3, 1);
            
            Drone.pos_ned_history = [];
            Drone.vel_xyz_history = [];
            
            Drone.path_len = 0;

            Drone.command = zeros(4, 1);
            Drone.prev_command = zeros(4, 1);
            Drone.full_command = zeros(19, 1);

            Drone.z_hat = zeros(22, 1);
            Drone.delta = zeros(4, 1);
            Drone.airdata = zeros(6, 1);

            Drone.forces = zeros(3, 1);
            Drone.moments = zeros(3, 1);

            Drone.body_handle = [];
            Drone.path_handle = [];
            Drone.waypoint_handle = [];
            cmap = jet(16);
            Drone.color = cmap(floor(rand*16)+1,:)';
            Drone.state_handle = [];

            Drone.altitude_state = 0;
            Drone.altitude_state_prev = 0;
            Drone.initialize_integrator = 0;
            Drone.lat_state = 0;
            Drone.lat_state_prev = 0;
            Drone.lat_init_integrator = 0;
            Drone.lat_counter_last_change = 0;

            Drone.i = zeros(5, 1);
            Drone.Q = p_battery.Q0;
            Drone.V = p_battery.V0;

            Drone.path = zeros(26, 1);
            Drone.alpha = 0;

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function init_rand_pos(self, map_size)
            % INIT_RAND_POS: Initialize the drone to a random position
            % on the map
            self.pos_ned = map_size .* rand(3, 1);
            self.pos_ned(3) = -self.pos_ned(3); % h = -pd
            % Update pos_ned_history
            if isempty(self.pos_ned_history)
                self.pos_ned_history = self.pos_ned';
            else
                self.pos_ned_history = [self.pos_ned_history; self.pos_ned'];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_pos(self, position_ned)
            % SET_POS: Set the position of the drone in the NED frame
            self.pos_ned = position_ned;
            % Update pos_ned_history
            if isempty(self.pos_ned_history)
                self.pos_ned_history = self.pos_ned';
            else
                self.pos_ned_history = [self.pos_ned_history; self.pos_ned'];
            end
            self.z_hat = true_states([self.get_state(); self.airdata; ...
                                       NaN]);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_vel(self, velocity_xyz)
            % SET_VEL: Set the velocity of the drone
            self.vel_xyz = velocity_xyz;
            if isempty(self.vel_xyz_history)
                self.vel_xyz_history = self.vel_xyz';
            else
                self.vel_xyz_history = [self.vel_xyz_history; self.vel_xyz'];
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function state = get_state(self)
            % GET_STATE: Get the state of the drone, vect12
            state = [self.pos_ned; self.vel_xyz; self.attitude; self.rates];
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function set_state(self, pos_ned, vel_xyz, attitude, rates)
            % SET_STATE: Set the drone state to the one passed in argument
            self.set_pos(self, pos_ned)
            self.set_vel(vel_xyz);
            self.attitude = attitude;
            self.rates = rates;

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function change_param(self, autopilot_version, p_drone)
            % CHANGE_PARAM: Change the drone parameters when changed from
            % GUI
            self.autopilot_version = autopilot_version;
            self.p_drone = p_drone;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_state(self, wind, time)
            % UPDATE_STATE: Compute the new drone state
            
            self.update_sensor_measurements();
            self.estimate_states(time);
            
            % Choose the autopilot
            if self.drone_type ~= "point_mass"
                if self.drone_type == "fixed_wing"
                    temp3 = autopilot_wing(self, 0, time);
                    
                elseif self.drone_type == "quadcopter"
                    temp3 = autopilot_quad(self, time);
                end
                
                self.delta = temp3(1:4);
                self.full_command = temp3(5:end);
                self.compute_dynamics(wind, time);
                self.compute_kinematics(time);
                self.update_battery();
                
            elseif self.drone_type == "point_mass"
                % Computes the new drone position with Euler forward method.
                % This method does not take the attitude into account.
                % We suppose that the attitude is always (0,0,0), so the
                % velocity in the body frame correponds to the velocity in the
                % inertial frame. Only usable with the velocity controller.
                
                self.vel_xyz = self.command(2:4);
                self.pos_ned = self.pos_ned + self.vel_xyz * self.p_sim.dt;
                self.attitude(3) = self.command(1); % to plot drone psi angle
                
                % Update pos_ned_history
                if isempty(self.pos_ned_history)
                    self.pos_ned_history = self.pos_ned';
                else
                    self.pos_ned_history = [self.pos_ned_history; self.pos_ned'];
                end
                
                % Update vel_xyz_history
                if isempty(self.vel_xyz_history)
                    self.vel_xyz_history = self.vel_xyz';
                else
                    self.vel_xyz_history = [self.vel_xyz_history; self.vel_xyz'];
                end
            end
            
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_prev_state(self)
            % UPDATE_PREV_STATE: Update the state variables of the drone, 
            % by replacing the old with the new ones.

            self.prev_state = self.get_state();
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_path_length(self)
            % UPDATE_PATH_LENGTH: Update the total lenght of the path that 
            % has been flyed by a drone
            self.path_len = self.path_len + ...
                pdist([self.prev_state(1:3)'; self.pos_ned'], 'euclidean');
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_battery(self)
            % UPDATE_BATTERY: Update the state variables of the battery
            % (i, V, Q)
            %
            % i : intensity         [A]
            % V : voltage           [V]
            % Q : consumed capacity [Ah]

            param = self.p_battery;
            % // TODO drone.update_actuators();
            omega = (self.p_drone.k_omega * self.delta) * 30 / pi; % [rpm]
            pow_motors = self.rpm2power(omega(1)) + self.rpm2power(omega(2)) + ...
                self.rpm2power(omega(3)) + self.rpm2power(omega(4));
            pow_tot = param.power_board + pow_motors;

            % Update current vector (i) with the new measurement
            self.i = circshift(self.i, -1);
            self.i(end) = pow_tot / self.V;

            % Moving-average filter on current (i)
            window_size = length(self.i);
            b = (1 / window_size) * ones(1, window_size);
            a = 1;
            signal_filt = filter(b, a, self.i);
            i_filt = signal_filt(end);

            self.Qt = (i_filt * self.p_sim.dt) / 3600;
            self.Q = self.Q + self.Qt; %[Ah]

            V_nominal = param.e0 + param.e1 * (self.Q / param.Qf) + ...
                param.e2 * (self.Q / param.Qf)^2;
            V_final = param.A * exp(-param.B * (param.Qf - self.Q));
            self.V = V_nominal - param.R * i_filt - V_final;

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function actuator_power = rpm2power(self, rpm)
            % Compute the power required from a motor from its velocity
            % expressed in rpm (round per minute)
            param = self.p_battery;
            actuator_power = param.p0 * (param.p1 * rpm^2 + param.p2 * rpm^4 + ...
                param.p3 * rpm^6 + param.p4 * rpm^8); %[W]
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function Qt = compute_Qt_hover(self)
            % Compute the capacity consumed in a time frame Ts for hovering
            omega = (self.get_delta_hover()) * 30 / pi; % [rpm]
            pow_motors = 4 * self.rpm2power(omega);
            pow_tot = self.B.pow_board + pow_motors;

            intensity = pow_tot / self.B.V0;

            Qt = (intensity * self.p_sim.dt) / 3600; %[Ah]
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_actuators(self)
            % UPDATE_ACTUATORS: Model for the motor dynamics

            % equations:  J omega_dot + b omega = K i
            %             L i_dot + R i = V - K omega

            % constants: to take from param or put in param if non-existent
            b = 0; % viscous constant
            L = 0.5e-3; % arm inductance
            R = 0.05; % arm resistance
            K = 4.2e-3; % EMF constant

            % // TO DO: differentiate between commanded delta and current
            % delta
            omega_old = self.delta * self.p_drone.k_omega;
            i_old = self.i(5);

            omega_dot = (-b * omega_old + K * i_old) / self.p_drone.J_prop;
            i_dot = (-K * omega_old - R * i_old + self.V) / L;

            self.delta = (omega_dot * self.p_sim.dt + omega_old) / self.p_drone.k_omega;
            self.i = i_dot * self.p_sim.dt + i_old;

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        plan_path(self, path_type, time)
        % PLAN_PATH: get waypoints

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function manage_path(self, time)
            % MANAGE_PATH: extract path from waypoints

            if self.drone_type == "fixed_wing"
                self.path = path_manager_wing([self.nb_waypoints; ...
                                            self.waypoints; self.z_hat; time], ...
                                            self.p_drone, self.p_sim);
            elseif self.drone_type == "quadcopter" || self.drone_type == "point_mass"
                self.path = path_manager_quad([self.nb_waypoints; ...
                                            self.waypoints; self.z_hat; time], ...
                                            self.p_drone, self.p_sim);
            end

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function follow_path(self) 
            % FOLLOW_PATH: get command from path

            if self.drone_type == "fixed_wing"
                self.command = path_follower_wing(self.path, self.p_drone);
            elseif self.drone_type == "quadcopter" || self.drone_type == "point_mass"
                self.command = path_follower_quad(self.path, self.p_drone);
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function delta_hover = get_delta_hover(self)
            delta_hover = sqrt((self.p_drone.mass * self.p_physics.gravity) / ...
                (self.p_drone.C_prop)) / (4 * self.p_drone.k_omega);
        end
        
        plot_state(self, time, output_rate)

    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods (Access = private)

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function compute_dynamics(self, wind, time)
            % COMPUTE_DYNAMICS: Compute forces and moments on the drone 
            % from its current state.

            if self.drone_type == "fixed_wing"
                out = forces_moments_wing(self.get_state(), self.delta, ...
                    wind, time, self.p_drone, self.p_physics);
            elseif self.drone_type == "quadcopter"
                out = forces_moments_quad(self.get_state(), self.delta, ...
                    wind, time, self.p_drone, self.p_sim, self.p_physics);
            end

            self.forces = out(1:3); % fx, fy, fz
            self.moments = out(4:6); % l, m, n
            self.airdata = out(7:12); % va, alpha, beta, wn, we, wd
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function compute_kinematics(self, time)
            % COMPUTE_KINEMATICS: Compute the new state of the drone from 
            % the forces and moments applied on it.

            self.update_prev_state();

            x_old = self.get_state();
            
            uu = [self.forces; self.moments];

            tspan = [time time+self.p_sim.dt];

            [~, x_ode] = ode23(@(t, x) kinematics_ode_fun(time, x, ...
                x_old, uu, self.p_drone), tspan, x_old);

            x_new = x_ode(end, :);
            self.pos_ned = x_new(1:3)';
            if isequal(self.pos_ned_history, [NaN, NaN, NaN])
                self.pos_ned_history = self.pos_ned;
            else
                self.pos_ned_history = [self.pos_ned_history; self.pos_ned'];
            end
            self.vel_xyz = x_new(4:6)';
            if isempty(self.vel_xyz_history) || isequal(self.vel_xyz_history, [NaN, NaN, NaN])
                self.vel_xyz_history = self.vel_xyz'; % rows = time steps, columns = states
            else
                self.vel_xyz_history = [self.vel_xyz_history; self.vel_xyz'];
            end
            self.attitude = x_new(7:9)';
            self.rates = x_new(10:12)';

            self.update_path_length();
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function update_sensor_measurements(self)
            % UPDATE_SENSOR_MEASUREMENTS: Update the sensor measurements
            % based on the sensor parameters.
            
            if self.use_estimation
                y_imu_baro = sensors(self.get_state(), self.forces, self.airdata, ...
                    self.p_drone, self.p_physics);
                y_gps = gps(self.get_state(), self.p_sim.dt, self.p_drone);

                self.y = [y_imu_baro; y_gps];
            end

        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function estimate_states(self, time)
            
            if self.use_estimation
                self.z_hat = estimate_states(self.y, time, self.p_sim.dt, ...
                    self.p_drone, self.p_physics); 
            else
                self.z_hat = true_states([self.get_state(); self.airdata; ...
                            time]);
            end
        end

    end

end
