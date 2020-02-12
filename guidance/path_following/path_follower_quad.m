function [command] = path_follower_quad(path, p_drone)

% PATH_FOLLOWER_QUAD - creates velocity command for the quadcopter
%
% Inputs:
%   flag   - if flag==1, follow waypoint path
%            if flag==2, follow orbit
%   
%   Va_d   - desired airspeed
%   r      - vect3, inertial position of start of waypoint path
%   q      - vect3, unit vector that defines inertial direction of waypoint path
%   r_next - inertial position of end of waypoint path
%   c      - vect3, center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%   xhat   - estimated MAV states (pn, pe, pd, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi)
%
% Outputs:
%

    NN = 0;
%     flag      = path(1+NN);         % 1 for line, 2 for orbit
    Va_d      = path(2+NN);         % base speed
    r_path    = path(3+NN:5+NN);    % inertial position of start of waypoint path
    q_path    = path(6+NN:8+NN);    % unit vector that defines inertial direction of waypoint path
    r_path_next = path(9+NN:11+NN); % inertial position of end of waypoint path
%     c_orbit   = path(9+NN:11+NN);   % center of orbit
%     rho_orbit = path(12+NN);        % radius of orbit
%     lambda_orbit = path(13+NN);     % direction of orbit (+1 for CW, -1 for CCW)
    NN = NN + 16;
    pn        = path(1+NN);
    pe        = path(2+NN);
    h         = path(3+NN);
%     Va        = in(4+NN);
%     alpha     = in(5+NN);
%     beta      = in(6+NN);
%     phi       = in(7+NN);
%     theta     = in(8+NN);
    chi       = path(9+NN);
%     p         = in(10+NN);
%     q         = in(11+NN);
%     r         = in(12+NN);
%     Vg        = in(13+NN);
%     wn        = in(14+NN);
%     we        = in(15+NN);
%     psi       = in(16+NN);
%     NN = NN + 16;
%     t         = in(1+NN);

%     flag = 1;   % only straight lines, valid for a quad
%     switch flag
%         case 1 % follow straight line path specified by r and q
            qn = q_path(1);
            qe = q_path(2);
            chi_q = atan2(qe, qn);

            % angle module 2pi
            while (chi_q - chi) < -pi
                chi_q = chi_q+2*pi;
            end
            while (chi_q - chi) > pi
                chi_q = chi_q-2*pi;
            end

            Rip = [cos(chi_q)     sin(chi_q)  0;
                 -sin(chi_q)    cos(chi_q)  0;
                 0              0           1];
            ep = Rip * ([pn; pe; -h] - r_path);   
            epy = ep(2);
            chi_c = chi_q - p_drone.chi_inf * 2/pi*atan(p_drone.k_path * epy);   
            h_c = -r_path_next(3);

    % command for quad velocity autopilot
    vn_c = Va_d * cos(chi_c);
    ve_c = Va_d * sin(chi_c);
    vd_c = (h-h_c);
    psi_c = chi_c;  % NOT TRUE! Ref. pag.21 Beard and McLain

    % create output
    command = [psi_c; vn_c; ve_c; vd_c];
end