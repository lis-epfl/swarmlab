function out = path_manager_quad_chap10(flag_line, pos_ned, p_drone)

% PATH_MANAGER_QUAD_CHAP10
%
% Outputs:
%   flag_line - if flag==1, follow waypoint path
%               if flag==2, follow orbit
%   
%   Va_d    - desired airspeed
%   r       - inertial position of start of waypoint path
%   q       - unit vector that defines inertial direction of waypoint path
%   c       - center of orbit
%   rho     - radius of orbit
%   lambda  - direction of orbit (+1 for CW, -1 for CCW)
%

    if flag_line == 1     % straight line
        Va_d     = p_drone.Va0;             % desired airspeed
        r        = [0; 0; -100];      % inertial position of line begin
        q        = [1/2; 1; 0];       % inertial direction of line
        q        = q/norm(q);         % inertial direction of line
        r_next   = [50; 100; -100];   % inertial position of line end
        c        = [0;0;0];           % not used for line
        rho      = 0;                 % not used for line
        lambda   = 0;                 % not used for line

    else                   % circular orbit
        % Define orbit
        flag_line= 2;
        Va_d     = p_drone.Va0;             % desired airspeed
        r        = [0; 0; 0];         % not used for orbit
        q        = [1; 0; 0];         % not used for orbit
        q        = q/norm(q);         % not used for orbit
        r_next   = [0; 0; 0];         % not used for orbit
        c        = [1;1;-100];        % center of orbit
        rho      = 400;               % radius of orbit
        lambda   = 1;                 % direction of orbit
    end

    out = [flag_line; Va_d; r; q; r_next; c; rho; lambda];

end