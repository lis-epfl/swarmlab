function y = autopilot_wing(drone, control_mode, t)
    % AUTOPILOT_WING - autopilot used for the fixed wing

    % State variables
    h = drone.z_hat(3); % altitude
    Va = drone.z_hat(7); % airspeed
    phi = drone.z_hat(10); % roll angle
    theta = drone.z_hat(11); % pitch angle
    chi = drone.z_hat(13); % course angle
    p = drone.z_hat(14); % body frame roll rate
    q = drone.z_hat(15); % body frame pitch rate
    r = drone.z_hat(16); % body frame yaw rate

    % Commands
    Va_c = drone.command(1); % commanded airspeed (m/s)
    h_c = drone.command(2); % commanded altitude (m)
    chi_c = drone.command(3); % commanded course (rad)

    autopilot_version = 2;
    % autopilot_version == 1 <- used for tuning
    % autopilot_version == 2 <- standard autopilot defined in book
    % autopilot_version == 3 <- Total Energy Control for longitudinal AP

    switch autopilot_version
        case 1
            [delta, x_command] = autopilot_tuning(Va_c, h_c, chi_c, Va, h, chi, ...
                phi, theta, p, q, r, t, drone);
        case 2
            [delta, x_command] = autopilot_uavbook(Va_c, h_c, chi_c, Va, h, chi, ...
                phi, theta, p, q, r, t, control_mode, drone);
        case 3
            [delta, x_command] = autopilot_TECS(Va_c, h_c, chi_c, Va, h, chi, phi, ...
                theta, p, q, r, t, drone);
    end

    y = [delta; x_command];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [delta, x_command] = autopilot_tuning(Va_c, h_c, chi_c, Va, h, chi, ...
        phi, theta, p, q, r, t, drone)
% AUTOPILOT_TUNING - used to tune each loop    
        
    %     global mode
    %     if isempty(mode)
    %         mode = 1;
    %     end

    mode = 5;

    switch mode
        case 1 % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, ...
                drone.p_sim, drone.P_phi_da);
            dr = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            de = drone.p_drone.de_T;
            dt = drone.p_drone.dt_T;
            theta_c = 0;
        case 2 % tune the course loop

            if t == 0
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 1, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
            else
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 0, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
            end

            [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, ...
                drone.p_sim, drone.P_phi_da);
            dr = 0; % no rudder
            % use trim values for elevator and throttle while tuning the
            % lateral autopilot
            de = drone.p_drone.de_T;
            dt = drone.p_drone.dt_T;
            theta_c = 0;
        case 3 % tune the throttle to airspeed loop and elevator to pitch
            % loop simultaneously
            theta_c = h_c;
            chi_c = 0;

            if t == 0
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 1, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
                [dt, drone.P_va_dt] = va_dt_hold(Va_c, Va, 1, drone.p_drone, drone.p_sim, drone.P_va_dt);
            else
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 0, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
                [dt, drone.P_va_dt] = va_dt_hold(Va_c, Va, 0, drone.p_drone, drone.p_sim, drone.P_va_dt);
            end

            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, drone.p_sim, ...
                drone.P_theta_de);
            [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, drone.p_sim, drone.P_phi_da);
            dr = 0; % no rudder
            % use trim values for elevator and throttle while tuning the
            % lateral autopilot
        case 4 % tune the pitch to airspeed loop
            chi_c = 0;
            dt = drone.p_drone.dt_T;

            if t == 0
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 1, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
                [theta_c, drone.P_va_theta] = va_theta_hold(Va_c, Va, 1, drone.p_drone, drone.p_sim,...
                    drone.P_va_theta);
            else
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 0, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
                [theta_c, drone.P_va_theta] = va_theta_hold(Va_c, Va, 0, drone.p_drone, drone.p_sim,...
                    drone.P_va_theta);
            end

            [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, drone.p_sim, drone.P_phi_da);
            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, drone.p_sim,...
                drone.P_theta_de);
            dr = 0; % no rudder
            % use trim values for elevator and throttle while tuning the
            % lateral autopilot
        case 5% tune the pitch to altitude loop
            %chi_c = 0;
            if t == 0
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 1, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
                [theta_c, drone.P_h_theta] = h_theta_hold(h_c, h, 1, drone.p_drone, ...
                    drone.p_sim, drone.P_h_theta);
                [dt, drone.P_va_dt] = va_dt_hold(Va_c, Va, 1, drone.p_drone, drone.p_sim, drone.P_va_dt);
            else
                [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 0, ...
                    drone.p_drone, drone.p_sim, drone.P_chi_phi);
                [theta_c, drone.P_h_theta] = h_theta_hold(h_c, h, 0, drone.p_drone, ...
                    drone.p_sim, drone.P_h_theta);
                [dt, drone.P_va_dt] = va_dt_hold(Va_c, Va, 0, drone.p_drone, drone.p_sim, drone.P_va_dt);
            end

            [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, drone.p_sim, drone.P_phi_da);
            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, drone.p_sim,...
                drone.P_theta_de);
            dr = 0; % no rudder
            % use trim values for elevator and throttle while tuning the
            % lateral autopilot
    end

    % Artificially saturation delta_t
    dt = saturate(dt, drone.p_drone.dt_max, 0);

    % Create control outputs
    delta = [de; da; dr; dt];

    % Commanded (desired) states
    x_command = [...
            0; ...% pn
            0; ...% pe
            h_c; ...% h
            0; ...% vn
            0; ...% ve
            0; ...% vd
            0; % an
            0; % ae
            0; % ad
            Va_c; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            0; % psi
            chi_c; ...% chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c, h_c, chi_c, Va, h, ...
        chi, phi, theta, p, q, r, t, control_mode, drone)
% AUTOPILOT_UAVBOOK - autopilot defined in the uavbook

    %persistent lat_state;
    %persistent lat_state_prev;
    %persistent lat_init_integrator;
    %persistent lat_counter_last_change;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Lateral autopilot
    drone.lat_state_prev = drone.lat_state;

    if (control_mode == 32 && drone.lat_state_prev == 1 && ...
            drone.lat_counter_last_change > 1)
        drone.lat_state = 2; % roll-hold
        drone.lat_counter_last_change = 0;
    elseif (control_mode == 32 && drone.lat_state_prev == 2 && ...
            drone.lat_counter_last_change > 1)
        drone.lat_state = 1; % course-hold
        drone.lat_counter_last_change = 0;
    elseif t == 0% initialisation
        drone.lat_state = 1;
    end

    if t == 0 || (drone.lat_state ~= drone.lat_state_prev)
        drone.lat_init_integrator = 1;
        drone.lat_counter_last_change = 0;
    else
        drone.lat_init_integrator = 0;
    end

    if drone.lat_state == 1
        [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, ...
            drone.lat_init_integrator, drone.p_drone, drone.p_sim, drone.P_chi_phi);
        [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, drone.p_sim, drone.P_phi_da);
    else
        phi_c = chi_c;

        if phi_c > deg2rad(30)
            phi_c = deg2rad(30);
        end

        if phi_c <- deg2rad(30)
            phi_c = -deg2rad(30);
        end

        [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, drone.p_sim, drone.P_phi_da);
    end

    % Assume no rudder, therefore set dr=0
    dr = 0; %coordinated_turn_hold(beta, 1, P);

    drone.lat_counter_last_change = drone.lat_counter_last_change + drone.p_sim.dt;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Longitudinal autopilot

    % define persistent variable for state of altitude state machine
    %persistent altitude_state;
    %persistent altitude_state_prev;
    %persistent initialize_integrator;

    % initialize persistent variable
    drone.altitude_state_prev = drone.altitude_state;

    if h <= drone.p_drone.altitude_take_off_zone
        drone.altitude_state = 1;
    elseif h <= h_c - drone.p_drone.h_theta_hold_zone
        drone.altitude_state = 2;
    elseif h >= h_c + drone.p_drone.h_theta_hold_zone
        drone.altitude_state = 3;
    else
        drone.altitude_state = 4;
    end

    if t == 0 || (drone.altitude_state ~= drone.altitude_state_prev)
        drone.initialize_integrator = 1;
    else
        drone.initialize_integrator = 0;
    end

    % Implement state machine

    switch drone.altitude_state
        case 1 % in take-off zone
            dt = 1;
            theta_c = deg2rad(30);
            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, ...
                drone.p_sim, drone.P_theta_de);
        case 2 % climb zone
            dt = 1;
            [theta_c, drone.P_va_theta] = va_theta_hold(Va_c, Va, ...
                drone.initialize_integrator, drone.p_drone, drone.p_sim, drone.P_va_theta);
            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, ...
                drone.p_sim, drone.P_theta_de);
        case 3 % descend zone
            dt = 0;
            [theta_c, drone.P_va_theta] = va_theta_hold(Va_c, Va, ...
                drone.initialize_integrator, drone.p_drone, drone.p_sim, drone.P_va_theta);
            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, ...
                drone.p_sim, drone.P_theta_de);
        case 4 % altitude hold zone
            [theta_c, drone.P_h_theta] = h_theta_hold(h_c, h, ...
                drone.initialize_integrator, drone.p_drone, drone.p_sim, drone.P_h_theta);
            [de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, ...
                drone.p_drone, drone.p_sim, drone.P_theta_de);
            [dt, drone.P_va_dt] = va_dt_hold(Va_c, Va, ...
                drone.initialize_integrator, drone.p_drone, drone.p_sim, drone.P_va_dt);
    end

    %[de, drone.P_theta_de] = theta_de_hold(theta_c, theta, q, drone.p_drone, drone.p_sim, drone.P_theta_de);
    % Artificially saturation delta_t
    dt = saturate(dt, drone.p_drone.dt_max, 0);

    % Create control outputs
    delta = [de; da; dr; dt];
    % Commanded (desired) states
    x_command = [...
                0; ...% pn
            0; ...% pe
            h_c; ...% h
            0; ...% vn
            0; ...% ve
            0; ...% vd
            0; % an
            0; % ae
            0; % ad
            Va_c; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            0; % psi
            chi_c; ...% chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c, h_c, chi_c, Va, h, ...
        chi, phi, theta, p, q, r, t, drone)
% AUTOPILOT_TECS - longitudinal autopilot based on total energy control systems

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Lateral autopilot
    if t == 0
        % assume no rudder, therefore set delta_r=0
        dr = 0; %coordinated_turn_hold(beta, 1, P);
        [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 1, drone.p_drone, drone.p_sim, drone.P_chi_phi);

    else
        [phi_c, drone.P_chi_phi] = chi_phi_hold(chi_c, chi, r, 0, drone.p_drone, drone.p_sim, drone.P_chi_phi);
        dr = 0; %coordinated_turn_hold(beta, 0, P);
    end

    [da, drone.P_phi_da] = phi_da_hold(phi_c, phi, p, drone.p_drone, drone.p_sim, drone.P_phi_da);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Longitudinal autopilot based on total energy control

    de = 0;
    dt = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create outputs

    % control outputs
    delta = [de; da; dr; dt];
    % commanded (desired) states
    x_command = [...
                0; ...% pn
            0; ...% pe
            h_c; ...% h
            0; ...% vn
            0; ...% ve
            0; ...% vd
            0; % an
            0; % ae
            0; % ad
            Va_c; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            0; % psi
            chi_c; ...% chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Lateral dynamics

function [da, P_phi_da] = phi_da_hold(phi_c, phi, p, p_drone, p_sim, P_phi_da)
    %persistent P_phi_da;

    if isempty(P_phi_da)
        P_phi_da.int = 0;
        P_phi_da.diff = 0;
        P_phi_da.error_prev = 0;
        P_phi_da.y_c_prev = 0;
    end

    % u = pid_loop(y_c, y, flag, kp, ki, kd, limit, Ts, tau)
    [da, P_phi_da] = pid_loop(phi_c, phi, Inf, p_drone.kp_phi, 0, p_drone.kd_phi, ...
        p_drone.da_max, p_sim.dt, p_drone.tau_phi, P_phi_da);
end

function [phi_c, P_chi_phi] = chi_phi_hold(chi_c, chi, r, reinit, p_drone, p_sim, P_chi_phi)
    %persistent P_chi_phi;

    if reinit || isempty(P_chi_phi)
        P_chi_phi.int = 0;
        P_chi_phi.diff = 0;
        P_chi_phi.error_prev = 0;
        P_chi_phi.y_c_prev = 0;
    end

    [phi_c, P_chi_phi] = pid_loop(chi_c, chi, r, p_drone.kp_chi, p_drone.ki_chi, 0, ...
        p_drone.phi_max, p_sim.dt, 0, P_chi_phi);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal dynamics

function [de, P_theta_de] = theta_de_hold(theta_c, theta, q, p_drone, p_sim, P_theta_de)
    %persistent P_theta_de;

    if isempty(P_theta_de)
        P_theta_de.int = 0;
        P_theta_de.diff = 0;
        P_theta_de.error_prev = 0;
        P_theta_de.y_c_prev = 0;
    end

    [de, P_theta_de] = pid_loop(theta_c, theta, NaN, p_drone.kp_theta, 0, p_drone.kd_theta, ...
        p_drone.de_max, p_sim.dt, p_drone.tau_theta, P_theta_de);
end

function [theta_c, P_h_theta] = h_theta_hold(h_c, h, reinit, p_drone, p_sim, P_h_theta)
    %persistent P_h_theta;

    if reinit || isempty(P_h_theta)
        P_h_theta.int = 0;
        P_h_theta.diff = 0;
        P_h_theta.error_prev = 0;
        P_h_theta.y_c_prev = 0;
    end

    [theta_c, P_h_theta] = pid_loop(h_c, h, NaN, p_drone.kp_h, p_drone.ki_h, 0, ...
        p_drone.theta_max, p_sim.dt, 0, P_h_theta);
end

function [dt, P_va_dt] = va_dt_hold(Va_c, Va, reinit, p_drone, p_sim, P_va_dt)
    %persistent P_va_dt;

    if reinit || isempty(P_va_dt)
        P_va_dt.int = 0;
        P_va_dt.diff = 0;
        P_va_dt.error_prev = 0;
        P_va_dt.y_c_prev = 0;
    end

    [dt, P_va_dt] = pid_loop(Va_c, Va, NaN, p_drone.kp_va_dt, p_drone.ki_va_dt, 0, ...
        p_drone.dt_max, p_sim.dt, 0, P_va_dt);
end

function [theta_c, P_va_theta] = va_theta_hold(Va_c, Va, reinit, p_drone, p_sim, P_va_theta)
    %persistent P_va_theta;

    if reinit || isempty(P_va_theta)
        P_va_theta.int = 0;
        P_va_theta.diff = 0;
        P_va_theta.error_prev = 0;
        P_va_theta.y_c_prev = 0;
    end

    [theta_c, P_va_theta] = pid_loop(Va_c, Va, NaN, p_drone.kp_va_theta, p_drone.ki_va_theta, 0, ...
        p_drone.theta_max, p_sim.dt, 0, P_va_theta);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Saturate - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function out = saturate(in, up_limit, low_limit)

    if in > up_limit
        out = up_limit;
    elseif in < low_limit
        out = low_limit;
    else
        out = in;
    end

end
