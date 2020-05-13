function y = autopilot_quad(drone, t)
    % AUTOPILOT_QUAD - autopilot used for the quadcopter

    p_drone = drone.p_drone;
    p_sim = drone.p_sim;
    autopilot_version = drone.autopilot_version;

    % State variables
    pn = drone.z_hat(1); % position north
    pe = drone.z_hat(2); % position east
    h = drone.z_hat(3); % altitude
    vx = drone.z_hat(4); % x velocity
    vy = drone.z_hat(5); % y velocity
    vz = drone.z_hat(6); % z velocity
    Va = drone.z_hat(7); % airspeed
    phi = drone.z_hat(10); % roll angle
    theta = drone.z_hat(11); % pitch angle
    psi = drone.z_hat(12); % yaw angle
    p = drone.z_hat(14); % body frame roll rate
    q = drone.z_hat(15); % body frame pitch rate
    r = drone.z_hat(16); % body frame yaw rate

    %%%%% Autopilot version %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % autopilot_version == 1 <- attitude controller
    % autopilot_version == 2 <- speed controller
    % autopilot_version == 3 <- acceleration controller
    % TO IMPLEMENT: autopilot_version == 4 <- position controller

    % Commanded values
    if autopilot_version == 1% attitude autopilot
        h_c = drone.command(1); % commanded altitude        [m]
        phi_c = drone.command(2); % commanded roll          [rad]
        theta_c = drone.command(3); % commanded pitch       [rad]
        psi_c = drone.command(4); % commanded yaw           [rad]

    elseif autopilot_version == 2% velocity autopilot
        psi_c = drone.command(1); % commanded yaw            [rad]
        vn_c = drone.command(2); % commanded north velocity  [m/s]
        ve_c = drone.command(3); % commanded east velocity   [m/s]
        vd_c = drone.command(4); % commanded down velocity   [m/s]

    elseif autopilot_version == 3% acceleration autopilot
        psi_c = drone.command(1); % commanded yaw            [rad]
        an_c = drone.command(2); % commanded north accell    [m/s^2]
        ae_c = drone.command(3); % commanded east accell     [m/s^2]
        ad_c = drone.command(4); % commanded down accell     [m/s^2]

    elseif autopilot_version == 4% position autopilot
        psi_c = drone.command(1); % commanded yaw            [rad]
        pn_c = drone.command(2); % commanded north position  [m]
        pe_c = drone.command(3); % commanded east position   [m]
        pd_c = drone.command(4); % commanded down position   [m]
    
    elseif autopilot_version == 5% cascaded velocity autopilot
        psi_c = drone.command(1); % commanded yaw            [rad]
        vn_c = drone.command(2); % commanded north velocity  [m/s]
        ve_c = drone.command(3); % commanded east velocity   [m/s]
        vd_c = drone.command(4); % commanded down velocity   [m/s]
    
    end

    %Forces
    F_xyz = drone.forces;

    a_ned = rotate_b2i(F_xyz, phi, theta, psi) / p_drone.mass;
    an = a_ned(1);
    ae = a_ned(2);
    ad = a_ned(3);

    switch autopilot_version
        case 1
            [delta, x_command] = autopilot_attitude(h_c, phi_c, theta_c, psi_c, ...
                h, phi, theta, psi, p, q, r, t, p_drone, p_sim, drone);

        case 2
            [delta, x_command] = autopilot_velocity(psi_c, vn_c, ve_c, vd_c, ...
                vx, vy, vz, phi, theta, psi, ...
                p, q, r, h, t, p_drone, p_sim, drone);
        case 3
            [delta, x_command] = autopilot_acceleration(psi_c, an_c, ae_c, ad_c, ...
                an, ae, ad, phi, theta, psi, ...
                p, q, r, h, t, p_drone, p_sim, drone);
        case 4
            [delta, x_command] = autopilot_position(psi_c, pn_c, pe_c, pd_c, ...
                vx, vy, vz, phi, theta, psi, ...
                p, q, r, h, pn, pe, t, p_drone, p_sim, drone);
        case 5    
            [delta, x_command] = autopilot_velocity_cascaded(psi_c, vn_c, ve_c, vd_c, ...
                vx, vy, vz, phi, theta, psi, ...
                p, q, r, h, t, p_drone, p_sim, drone);
    end

    y = [delta; x_command];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_attitude(h_c, phi_c, theta_c, ...
        psi_c, h, phi, theta, psi, p, q, r, t, p_drone, p_sim, drone)
% AUTOPILOT_ATTITUDE

    if h <= p_drone.altitude_take_off_zone
        drone.altitude_state = 2;
    else
        drone.altitude_state = 2;
    end

    if t == 0 || (drone.altitude_state ~= drone.altitude_state_prev)
        drone.initialize_integrator = 1;
    else
        drone.initialize_integrator = 0;
    end

    % Implement state machine
    switch drone.altitude_state
        case 1% in take-off zone
            d1 = 1;
            d2 = 1;
            d3 = 1;
            d4 = 1;

        case 2% outside take-off zone
            [thrust, drone.P_h_thrust] = h_thrust_hold(h_c, h, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_h_thrust);
            [torque_phi, drone.P_roll_torque] = roll_torque_hold(phi_c, ...
                phi, p / 180 * pi, drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_roll_torque);
            [torque_theta, drone.P_pitch_torque] = pitch_torque_hold(...
                theta_c, theta, q / 180 * pi, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_pitch_torque);
            [torque_psi, drone.P_psi_torque] = yaw_torque_hold(psi_c, ...
                psi, r / 180 * pi, drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_psi_torque);

            [d1, d2, d3, d4] = mixer(thrust, torque_phi, torque_theta, ...
                torque_psi, p_drone, p_sim);

            % artificially saturation delta_t
            d1 = saturate(d1, p_drone.dt_max, 0);
            d2 = saturate(d2, p_drone.dt_max, 0);
            d3 = saturate(d3, p_drone.dt_max, 0);
            d4 = saturate(d4, p_drone.dt_max, 0);
    end

    % Control outputs
    delta = [d1; d2; d3; d4];

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
            0; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            psi_c; ...% psi
            0; % chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];
    drone.altitude_state_prev = drone.altitude_state;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_velocity_cascaded(psi_c, vn_c, ve_c, ...
        vd_c, vx, vy, vz, phi, theta, psi, p, q, r, h, t, p_drone, p_sim, drone)
% AUTOPILOT_VELOCITY

    Rbi = Rb2i(phi, theta, psi);
    v_ned = Rbi * [vx vy vz]';
    vn = v_ned(1);
    ve = v_ned(2);
    vd = v_ned(3);

    if h <= p_drone.altitude_take_off_zone
        drone.altitude_state = 2;
    else
        drone.altitude_state = 2;
    end

    if t == 0 || (drone.altitude_state ~= drone.altitude_state_prev)
        drone.initialize_integrator = 1;
    else
        drone.initialize_integrator = 0;
    end

    % Implement state machine
    switch drone.altitude_state
        case 1% in take-off zone
            d1 = 1;
            d2 = 1;
            d3 = 1;
            d4 = 1;
        case 2% outside take-off zone
            [thrust, drone.P_vd_thrust] = vd_thrust_hold(vd_c, vd, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_vd_thrust);
            [phi_c, drone.P_ve_roll] = ve_roll_hold(ve_c, ve, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_ve_roll);
            [p_c, drone.P_roll_p] = roll_p_hold(phi_c, ...
                phi, p / 180 * pi, drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_roll_p);
            [torque_phi, drone.P_p_torque] = p_torque_hold(p_c, ...
                p, drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_p_torque);
            [theta_c, drone.P_vn_pitch] = vn_pitch_hold(vn_c, vn, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_vn_pitch);
            [q_c, drone.P_pitch_q] = pitch_q_hold(...
                theta_c, theta, q / 180 * pi, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_pitch_q);
            [torque_theta, drone.P_q_torque] = q_torque_hold(q_c, ...
                q, drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_q_torque);
            [r_c, drone.P_yaw_r] = yaw_r_hold(psi_c, ...
                psi, r / 180 * pi, drone.initialize_integrator, p_drone, p_sim,...
                drone.P_yaw_r);
            [torque_psi, drone.P_r_torque] = r_torque_hold(r_c, ...
                r, drone.initialize_integrator, p_drone, p_sim,...
                drone.P_r_torque);

            [d1, d2, d3, d4] = mixer(thrust, torque_phi, torque_theta, ...
                torque_psi, p_drone, p_sim);

            % artificially saturation delta_t
            d1 = saturate(d1, p_drone.dt_max, 0);
            d2 = saturate(d2, p_drone.dt_max, 0);
            d3 = saturate(d3, p_drone.dt_max, 0);
            d4 = saturate(d4, p_drone.dt_max, 0);
    end

    % Control outputs
    delta = [d1; d2; d3; d4];

    % Commanded (desired) states
    x_command = [...
                0; ...% pn
            0; ...% pe
            0; ...% h
            vn_c; ...% vn
            ve_c; ...% ve
            vd_c; ...% vd
            0; % an
            0; % ae
            0; % ad
            0; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            psi_c; ...% psi
            0; % chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];
    drone.altitude_state_prev = drone.altitude_state;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_velocity(psi_c, vn_c, ve_c, ...
        vd_c, vx, vy, vz, phi, theta, psi, p, q, r, h, t, p_drone, p_sim, drone)
% AUTOPILOT_VELOCITY

    Rbi = Rb2i(phi, theta, psi);
    v_ned = Rbi * [vx vy vz]';
    vn = v_ned(1);
    ve = v_ned(2);
    vd = v_ned(3);

    if h <= p_drone.altitude_take_off_zone
        drone.altitude_state = 2;
    else
        drone.altitude_state = 2;
    end

    if t == 0 || (drone.altitude_state ~= drone.altitude_state_prev)
        drone.initialize_integrator = 1;
    else
        drone.initialize_integrator = 0;
    end

    % Implement state machine
    switch drone.altitude_state
        case 1% in take-off zone
            d1 = 1;
            d2 = 1;
            d3 = 1;
            d4 = 1;
        case 2% outside take-off zone
            [thrust, drone.P_vd_thrust] = vd_thrust_hold(vd_c, vd, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_vd_thrust);
            [phi_c, drone.P_ve_roll] = ve_roll_hold(ve_c, ve, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_ve_roll);
            [torque_phi, drone.P_roll_torque] = roll_torque_hold(phi_c, ...
                phi, p / 180 * pi, drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_roll_torque);
            [theta_c, drone.P_vn_pitch] = vn_pitch_hold(vn_c, vn, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_vn_pitch);
            [torque_theta, drone.P_pitch_torque] = pitch_torque_hold(...
                theta_c, theta, q / 180 * pi, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_pitch_torque);
            [torque_psi, drone.P_psi_torque] = yaw_torque_hold(psi_c, ...
                psi, r / 180 * pi, drone.initialize_integrator, p_drone, p_sim,...
                drone.P_psi_torque);

            [d1, d2, d3, d4] = mixer(thrust, torque_phi, torque_theta, ...
                torque_psi, p_drone, p_sim);

            % artificially saturation delta_t
            d1 = saturate(d1, p_drone.dt_max, 0);
            d2 = saturate(d2, p_drone.dt_max, 0);
            d3 = saturate(d3, p_drone.dt_max, 0);
            d4 = saturate(d4, p_drone.dt_max, 0);
    end

    % Control outputs
    delta = [d1; d2; d3; d4];

    % Commanded (desired) states
    x_command = [...
                0; ...% pn
            0; ...% pe
            0; ...% h
            vn_c; ...% vn
            ve_c; ...% ve
            vd_c; ...% vd
            0; % an
            0; % ae
            0; % ad
            0; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            psi_c; ...% psi
            0; % chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];
    drone.altitude_state_prev = drone.altitude_state;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% // TODO: Check it
function [delta, x_command] = autopilot_acceleration(psi_c, an_c, ...
        ae_c, ad_c, an, ae, ad, phi, theta, psi, p, q, r, h, t, p_drone, p_sim, drone)
% AUTOPILOT_ACCELERATION

    if h <= p_drone.altitude_take_off_zone
        drone.altitude_state = 2;
    else
        drone.altitude_state = 2;
    end

    if t == 0 || (drone.altitude_state ~= drone.altitude_state_prev)
        drone.initialize_integrator = 1;
    else
        drone.initialize_integrator = 0;
    end

    % Implement state machine
    switch drone.altitude_state
        case 1% in take-off zone
            d1 = 1;
            d2 = 1;
            d3 = 1;
            d4 = 1;

        case 2% outside take-off zone
            [thrust, drone.P_ad_thrust] = ad_thrust_hold(ad_c, ad, ...
                drone.initialize_integrator, p_drone, drone.P_ad_thrust);
            %[thrust, drone.P_h_thrust] = h_thrust_hold(15, h, ...
                % drone.initialize_integrator, p_drone, drone.P_h_thrust);
            [phi_c, drone.P_ae_roll] = ae_roll_hold(ae_c, ae, ...
                drone.initialize_integrator, p_drone, drone.P_ae_roll);
            [torque_phi, drone.P_roll_torque] = roll_torque_hold(phi_c, ...
                phi, deg2rad(p), drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_roll_torque);
            [theta_c, drone.P_an_pitch] = an_pitch_hold(an_c, an, ...
                drone.initialize_integrator, p_drone, drone.P_an_pitch);
            [torque_theta, drone.P_pitch_torque] = pitch_torque_hold(...
                theta_c, theta, deg2rad(q), ...
                drone.initialize_integrator, p_drone, drone.P_pitch_torque);
            [torque_psi, drone.P_psi_torque] = yaw_torque_hold(psi_c, ...
                psi, deg2rad(r), drone.initialize_integrator, p_drone, p_sim,  ...
                drone.P_psi_torque);

            [d1, d2, d3, d4] = mixer(thrust, torque_phi, torque_theta, ...
                torque_psi, p_drone, p_sim);

            % artificially saturation delta_t
            d1 = saturate(d1, p_drone.dt_max, 0);
            d2 = saturate(d2, p_drone.dt_max, 0);
            d3 = saturate(d3, p_drone.dt_max, 0);
            d4 = saturate(d4, p_drone.dt_max, 0);
    end

    % Control outputs
    delta = [d1; d2; d3; d4];

    % Commanded (desired) states
    x_command = [...
                0; ...% pn
            0; ...% pe
            0; ...% h
            0; ...% vn
            0; ...% ve
            0; ...% vd
            an_c;
            ae_c;
            ad_c;
            0; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            psi_c; ...% psi
            0; % chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];
    drone.altitude_state_prev = drone.altitude_state;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_position(psi_c, pn_c, pe_c, ...
        pd_c, vx, vy, vz, phi, theta, psi, p, q, r, h, pn, pe, t, p_drone, p_sim, drone)
% AUTOPILOT_POSITION
% // TODO: Check it
    Rbi = Rb2i(phi, theta, psi);
    v_ned = Rbi * [vx vy vz]';
    vn = v_ned(1);
    ve = v_ned(2);
    vd = v_ned(3);

    if h <= p_drone.altitude_take_off_zone
        drone.altitude_state = 2;
    else
        drone.altitude_state = 2;
    end

    if t == 0 || (drone.altitude_state ~= drone.altitude_state_prev)
        drone.initialize_integrator = 1;
    else
        drone.initialize_integrator = 0;
    end

    % Implement state machine
    switch drone.altitude_state
        case 1% in take-off zone
            d1 = 1;
            d2 = 1;
            d3 = 1;
            d4 = 1;
        case 2% outside take-off zone
            [vd_c, drone.P_pd_thrust] = pd_vd_hold(pd_c, h, ...
                drone.initialize_integrator, p_drone, drone.P_pd_thrust);
            [thrust, drone.P_vd_thrust] = vd_thrust_hold(vd_c, vd, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_vd_thrust);
            [ve_c, drone.P_pe_roll] = pe_ve_hold(pe_c, pe, ...
                drone.initialize_integrator, p_drone, drone.P_pe_thrust);
            [phi_c, drone.P_ve_roll] = ve_roll_hold(ve_c, ve, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_ve_roll);
            [torque_phi, drone.P_roll_torque] = roll_torque_hold(phi_c, ...
                phi, deg2rad(p), drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_roll_torque);
            [vn_c, drone.P_pn_pitch] = pn_vn_hold(pn_c, pn, ...
                drone.initialize_integrator, p_drone, drone.P_pn_pitch);
            [theta_c, drone.P_vn_pitch] = vn_pitch_hold(vn_c, vn, ...
                drone.initialize_integrator, p_drone, p_sim, drone.P_vn_pitch);
            [torque_theta, drone.P_pitch_torque] = pitch_torque_hold(...
                theta_c, theta, deg2rad(q), ...
                drone.initialize_integrator, p_drone, drone.P_pitch_torque);
            [torque_psi, drone.P_psi_torque] = yaw_torque_hold(psi_c, ...
                psi, deg2rad(r), drone.initialize_integrator, p_drone, p_sim, ...
                drone.P_psi_torque);

            [d1, d2, d3, d4] = mixer(thrust, torque_phi, torque_theta, ...
                torque_psi, p_drone, p_sim);

            % artificially saturation delta_t
            d1 = saturate(d1, p_drone.dt_max, 0);
            d2 = saturate(d2, p_drone.dt_max, 0);
            d3 = saturate(d3, p_drone.dt_max, 0);
            d4 = saturate(d4, p_drone.dt_max, 0);
    end

    % Control outputs
    delta = [d1; d2; d3; d4];

    % Commanded (desired) states
    x_command = [...
                pn_c; ...% pn
            pe_c; ...% pe
            pd_c; ...% h
            vn_c; ...% vn
            ve_c; ...% ve
            vd_c; ...% vd
            0; % an
            0; % ae
            0; % ad
            0; ...% Va
            0; ...% alpha
            0; ...% beta
            phi_c; ...% phi
            theta_c; % theta
            psi_c; ...% psi
            0; % chi
            0; ...% p
            0; ...% q
            0; ...% r
            ];

    y = [delta; x_command];
    drone.altitude_state_prev = drone.altitude_state;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vd_c, P_pd_thrust] = pd_vd_hold(pd_c, pd, reinit, p_drone, p_sim, P_pd_thrust)

    if reinit || isempty(P_pd_thrust)
        P_pd_thrust.int = 0;
        P_pd_thrust.diff = 0;
        P_pd_thrust.error_prev = 0;
        P_pd_thrust.y_c_prev = 0;
    end

    [vd_c, P_pd_thrust] = pid_loop(pd_c, pd, Inf, p_drone.kp_vd, p_drone.ki_vd, p_drone.kd_vd, ...
        p_drone.vd_max, p_sim.dt, 0.1, P_pd_thrust);
end

function [ve_c, P_pe_roll] = pe_ve_hold(pe_c, pe, reinit, p_drone, P_pe_roll)

    if reinit || isempty(P_pe_roll)
        P_pe_roll.int = 0;
        P_pe_roll.diff = 0;
        P_pe_roll.error_prev = 0;
        P_pe_roll.y_c_prev = 0;
    end

    [ve_c, P_pe_roll] = pid_loop(pe_c, pe, Inf, p_drone.kp_ve, p_drone.ki_ve, p_drone.kd_ve, ...
        p_drone.v_max, p_sim.dt, 0.1, P_pe_roll);
end

function [vn_c, P_pn_pitch] = pn_vn_hold(pn_c, pn, reinit, p_drone, p_sim, P_pn_pitch)

    if reinit || isempty(P_pn_pitch)
        P_pn_pitch.int = 0;
        P_pn_pitch.diff = 0;
        P_pn_pitch.error_prev = 0;
        P_pn_pitch.y_c_prev = 0;
    end

    [vn_c, P_pn_pitch] = pid_loop(pn_c, pn, Inf, p_drone.kp_vn, p_drone.ki_vn, p_drone.kd_vn, ...
        p_drone.v_max, p_sim.dt, 0.1, P_pn_pitch);
end

function [thrust, P_h_thrust] = h_thrust_hold(h_c, h, reinit, p_drone, p_sim, P_h_thrust)

    if reinit || isempty(P_h_thrust)
        P_h_thrust.int = 0;
        P_h_thrust.diff = 0;
        P_h_thrust.error_prev = 0;
        P_h_thrust.y_c_prev = 0;
    end

    [thrust, P_h_thrust] = pid_loop(h_c, h, Inf, p_drone.kp_h, p_drone.ki_h, p_drone.kd_h, ...
        p_drone.thrust_max, p_sim.dt, 0.1, P_h_thrust);
end

function [torque_phi, P_roll_torque] = roll_torque_hold(phi_c, phi, p, ...
        reinit, p_drone, p_sim, P_roll_torque)

    if reinit || isempty(P_roll_torque)
        P_roll_torque.int = 0;
        P_roll_torque.diff = 0;
        P_roll_torque.error_prev = 0;
        P_roll_torque.y_c_prev = 0;
    end

    [torque_phi, P_roll_torque] = pid_loop(phi_c, phi, p, p_drone.kp_phi, 0, ...
        p_drone.kd_phi, p_drone.torque_max, p_sim.dt, 0.1, P_roll_torque);
end

function [torque_theta, P_pitch_torque] = pitch_torque_hold(theta_c, ...
        theta, q, reinit, p_drone, p_sim, P_pitch_torque)

    if reinit || isempty(P_pitch_torque)
        P_pitch_torque.int = 0;
        P_pitch_torque.diff = 0;
        P_pitch_torque.error_prev = 0;
        P_pitch_torque.y_c_prev = 0;
    end

    [torque_theta, P_pitch_torque] = pid_loop(theta_c, theta, q, ...
        p_drone.kp_theta, 0, p_drone.kd_theta, p_drone.torque_max, p_sim.dt, 0.1, ...
        P_pitch_torque);
end

function [torque_psi, P_psi_torque] = yaw_torque_hold(psi_c, psi, r, ...
        reinit, p_drone, p_sim, P_psi_torque)

    if reinit || isempty(P_psi_torque)
        P_psi_torque.int = 0;
        P_psi_torque.diff = 0;
        P_psi_torque.error_prev = 0;
        P_psi_torque.y_c_prev = 0;
    end

    [torque_psi, P_psi_torque] = pid_loop(psi_c, psi, r, p_drone.kp_psi, 0, ...
        p_drone.kd_psi, p_drone.torque_max, p_sim.dt, 0.1, P_psi_torque);
end

function [theta_c, P_vn_pitch] = vn_pitch_hold(vn_c, vn, reinit, p_drone, p_sim, ...
        P_vn_pitch)

    if reinit || isempty(P_vn_pitch)
        P_vn_pitch.int = 0;
        P_vn_pitch.diff = 0;
        P_vn_pitch.error_prev = 0;
        P_vn_pitch.y_c_prev = 0;
    end

    [theta_c, P_vn_pitch] = pid_loop(vn_c, vn, Inf, p_drone.kp_vn, p_drone.ki_vn, 0, ...
        p_drone.theta_max, p_sim.dt, 0.1, P_vn_pitch);
end

function [phi_c, P_ve_roll] = ve_roll_hold(ve_c, ve, reinit, p_drone, p_sim, P_ve_roll)

    if reinit || isempty(P_ve_roll)
        P_ve_roll.int = 0;
        P_ve_roll.diff = 0;
        P_ve_roll.error_prev = 0;
        P_ve_roll.y_c_prev = 0;
    end

    [phi_c, P_ve_roll] = pid_loop(ve_c, ve, Inf, p_drone.kp_ve, p_drone.ki_ve, 0, ...
        p_drone.phi_max, p_sim.dt, 0.1, P_ve_roll);
end

function [thrust, P_vd_thrust] = vd_thrust_hold(vd_c, vd, reinit, p_drone, p_sim, ...
        P_vd_thrust)

    if reinit || isempty(P_vd_thrust)
        % P_vd_thrust.int = -0.5;
        P_vd_thrust.int = 0;
        P_vd_thrust.diff = 0;
        P_vd_thrust.error_prev = 0;
        P_vd_thrust.y_c_prev = 0;
    end

    [thrust, P_vd_thrust] = pid_loop(vd_c, vd, Inf, p_drone.kp_vd, p_drone.ki_vd, 0, ...
        p_drone.thrust_max, p_sim.dt, 0.1, P_vd_thrust);
end

function [p_c, P_roll_p] = roll_p_hold(phi_c, ...
    phi, p, reinit, p_drone, p_sim, P_roll_p)

    if reinit || isempty(P_roll_p)
        P_roll_p.int = 0;
        P_roll_p.diff = 0;
        P_roll_p.error_prev = 0;
        P_roll_p.y_c_prev = 0;
    end

    [p_c, P_roll_p] = pid_loop(phi_c, phi, p, p_drone.kp_phi_cas, ...
        p_drone.ki_phi_cas, p_drone.kd_phi_cas, ...
        p_drone.p_max, p_sim.dt, 0.1, P_roll_p);
end

function [q_c, P_pitch_q] = pitch_q_hold(theta_c, ...
    theta, q, reinit, p_drone, p_sim, P_pitch_q)

    if reinit || isempty(P_pitch_q)
        P_pitch_q.int = 0;
        P_pitch_q.diff = 0;
        P_pitch_q.error_prev = 0;
        P_pitch_q.y_c_prev = 0;
    end

    [q_c, P_pitch_q] = pid_loop(theta_c, theta, q, p_drone.kp_theta_cas, ...
        p_drone.ki_theta_cas, p_drone.kd_theta_cas, ...
        p_drone.q_max, p_sim.dt, 0.1, P_pitch_q);
end

function [r_c, P_yaw_r] = yaw_r_hold(psi_c, ...
    psi, r, reinit, p_drone, p_sim, P_yaw_r)

    if reinit || isempty(P_yaw_r)
        P_yaw_r.int = 0;
        P_yaw_r.diff = 0;
        P_yaw_r.error_prev = 0;
        P_yaw_r.y_c_prev = 0;
    end

    [r_c, P_yaw_r] = pid_loop(psi_c, psi, r, p_drone.kp_psi_cas, ...
        p_drone.ki_psi_cas, p_drone.kd_psi_cas, ...
        p_drone.r_max, p_sim.dt, 0.1, P_yaw_r);
end

function [torque_phi, P_p_torque] = p_torque_hold(p_c, ...
    p, reinit, p_drone, p_sim, P_p_torque)
    
    if reinit || isempty(P_p_torque)
        P_p_torque.int = 0;
        P_p_torque.diff = 0;
        P_p_torque.error_prev = 0;
        P_p_torque.y_c_prev = 0;
    end

    [torque_phi, P_p_torque] = pid_loop(p_c, p, Inf, p_drone.kp_p, p_drone.ki_p, p_drone.kd_p, ...
        p_drone.torque_max, p_sim.dt, 0.1, P_p_torque);

end

function [torque_theta, P_q_torque] = q_torque_hold(q_c, ...
    q, reinit, p_drone, p_sim, P_q_torque)
    
    if reinit || isempty(P_q_torque)
        P_q_torque.int = 0;
        P_q_torque.diff = 0;
        P_q_torque.error_prev = 0;
        P_q_torque.y_c_prev = 0;
    end

    [torque_theta, P_q_torque] = pid_loop(q_c, q, Inf, p_drone.kp_q, p_drone.ki_q, p_drone.kd_q, ...
        p_drone.torque_max, p_sim.dt, 0.1, P_q_torque);

end

function [torque_psi, P_r_torque] = r_torque_hold(r_c, ...
    r, reinit, p_drone, p_sim, P_r_torque)
    
    if reinit || isempty(P_r_torque)
        P_r_torque.int = 0;
        P_r_torque.diff = 0;
        P_r_torque.error_prev = 0;
        P_r_torque.y_c_prev = 0;
    end

    [torque_psi, P_r_torque] = pid_loop(r_c, r, Inf, p_drone.kp_r, p_drone.ki_r, p_drone.kd_r, ...
        p_drone.torque_max, p_sim.dt, 0.1, P_r_torque);

end

function [thrust, P_ad_thrust] = ad_thrust_hold(ad_c, ad, reinit, p_drone, p_sim, ...
        P_ad_thrust)

    if reinit || isempty(P_ad_thrust)
        P_ad_thrust.int = 0;
        P_ad_thrust.diff = 0;
        P_ad_thrust.error_prev = 0;
        P_ad_thrust.y_c_prev = 0;
    end

    [thrust, P_ad_thrust] = pid_loop(ad_c, ad, Inf, p_drone.kp_ad, p_drone.ki_ad, 0, ...
        p_drone.thrust_max, p_sim.dt, 0.1, P_ad_thrust);
end

function [theta_c, P_an_pitch] = an_pitch_hold(an_c, an, reinit, p_drone, ...
        P_an_pitch)

    if reinit || isempty(P_an_pitch)
        P_an_pitch.int = 0;
        P_an_pitch.diff = 0;
        P_an_pitch.error_prev = 0;
        P_an_pitch.y_c_prev = 0;
    end

    [theta_c, P_an_pitch] = pid_loop(an_c, an, Inf, p_drone.kp_an, p_drone.ki_an, 0, ...
        p_drone.theta_max, p_sim.dt, 0.1, P_an_pitch);
end

function [phi_c, P_ae_roll] = ae_roll_hold(ae_c, ae, reinit, p_drone, P_ae_roll)

    if reinit || isempty(P_ae_roll)
        P_ae_roll.int = 0;
        P_ae_roll.diff = 0;
        P_ae_roll.error_prev = 0;
        P_ae_roll.y_c_prev = 0;
    end

    [phi_c, P_ae_roll] = pid_loop(ae_c, ae, Inf, p_drone.kp_ae, p_drone.ki_ae, 0, ...
        p_drone.phi_max, p_sim.dt, 0.1, P_ae_roll);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mixer
%     - mixes the effects of thrust and torques
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [d1, d2, d3, d4] = mixer(thrust, torque_phi, torque_theta, ...
        torque_psi, p_drone, p_sim)

    omega1_sqd = (thrust / (4 * p_drone.C_prop) + ...
        torque_theta / (2 * p_drone.C_prop * p_drone.l_arm) - ...
        torque_psi / (4 * p_drone.CD));

    if omega1_sqd > 0
        d1 = sqrt(omega1_sqd) / p_drone.k_omega;
    else
        d1 = 0;
    end

    omega2_sqd = (thrust / (4 * p_drone.C_prop) - ...
        torque_phi / (2 * p_drone.C_prop * p_drone.l_arm) + ...
        torque_psi / (4 * p_drone.CD));

    if omega2_sqd > 0
        d2 = sqrt(omega2_sqd) / p_drone.k_omega;
    else
        d2 = 0;
    end

    omega3_sqd = (thrust / (4 * p_drone.C_prop) - ...
        torque_theta / (2 * p_drone.C_prop * p_drone.l_arm) - ...
        torque_psi / (4 * p_drone.CD));

    if omega3_sqd > 0
        d3 = sqrt(omega3_sqd) / p_drone.k_omega;
    else
        d3 = 0;
    end

    omega4_sqd = (thrust / (4 * p_drone.C_prop) + ...
        torque_phi / (2 * p_drone.C_prop * p_drone.l_arm) + ...
        torque_psi / (4 * p_drone.CD));

    if omega4_sqd > 0
        d4 = sqrt(omega4_sqd) / p_drone.k_omega;
    else
        d4 = 0;
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Saturate
%       - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = saturate(in, up_limit, low_limit)

    if in > up_limit
        out = up_limit;
    elseif in < low_limit
        out = low_limit;
    else
        out = in;
    end

end
