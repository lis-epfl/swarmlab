function out = path_manager_quad(in,p_drone, p_sim)

% PATH_MANAGER_QUAD
%
% Inputs:
%   nb_wpts      - number of waypoint configurations
%   wpt_list     - an array of dimension 5 by p_drone.size_wpt_array.
%                - the first nb_wpts rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, chi_d, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, chi_d is the desired course at the waypoint,
%                  and Va_d is the desired airspeed along the path.  If
%                  abs(chi_d)<2*pi then Dubins paths will be followed
%                  between waypoint configurations.  If abs(chi_d)>=2*pi
%                  then straight-line paths (with fillets) are commanded.
%
% Outputs:
%   flag    - if flag==1, follow waypoint path
%             if flag==2, follow orbit 
%   Va_d    - desired airspeed
%   r       - inertial position of start of waypoint path
%   q       - unit vector that defines inertial direction of waypoint path
%   r_next  - inertial position of end of waypoint path
%   c       - center of orbit
%   rho     - radius of orbit
%   lambda  - direction of orbit (+1 for CW, -1 for CCW)
%   state   - drone state
%   flag_new_wpt  - demand new waypoints from path planer
%

    persistent start_of_simulation

    time = in(end);
    if time == 0 || time == p_sim.dt
      start_of_simulation = 1;
    end

    NN = 0;
    nb_wpts = in(1+NN);
    if nb_wpts == 0 % start of simulation
        flag   = 1; % following straight line path
        Va_d   = p_drone.Va0; % desired airspeed along waypoint path
        NN = NN + 1 + 5*p_drone.size_wpt_array;
        pn        = in(1+NN);
        pe        = in(2+NN);
        h         = in(3+NN);
        chi       = in(9+NN);
        r         = [pn; pe; -h];
        q         = [cos(chi); sin(chi); 0];
        c         = [0; 0; 0];
        rho       = p_drone.radius;
        lambda    = 0;
        state     =  in(1+NN:16+NN);
        flag_need_new_wpts = 1;
        out = [flag; Va_d; r; q; r; c; rho; lambda; state; flag_need_new_wpts];
    else
        out = path_manager_fillet_quad(in,p_drone,start_of_simulation);  % smooths through waypoints with fillets
        start_of_simulation=0;
    end

end


function out = path_manager_fillet_quad(in, p_drone, start_of_simulation)
% PATH_MANAGER_FILLET_QUAD - follow lines between waypoints.  Smooth 
% transition with fillets.
%
% Inputs:
%   nb_wpts     - number of waypoint configurations
%   wpt_list    - an array of dimension 5 by p_drone.size_wpt_array.
%               - the first nb_wpts rows define waypoint
%                 configurations
%               - format for each waypoint configuration:
%                 [wn, we, wd, dont_care, Va_d]
%                 where the (wn, we, wd) is the NED position of the
%                 waypoint, and Va_d is the desired airspeed along the
%                 path.
%
% Outputs:
%   flag  - if flag==1, follow waypoint path
%           if flag==2, follow orbit
%   
%   Va_d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   r_next - inertial position of end of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%

    NN = 0;
    nb_wpts = in(1+NN);
    wpt_list = reshape(in(2+NN:5*p_drone.size_wpt_array+1+NN),5,p_drone.size_wpt_array);
    NN = NN + 1 + 5*p_drone.size_wpt_array;
    pn        = in(1+NN);
    pe        = in(2+NN);
    h         = in(3+NN);
    % Va      = in(4+NN);
    % alpha   = in(5+NN);
    % beta    = in(6+NN);
    % phi     = in(7+NN);
    % theta   = in(8+NN);
    % chi     = in(9+NN);
    % p       = in(10+NN);
    % q       = in(11+NN);
    % r       = in(12+NN);
    % Vg      = in(13+NN);
    % wn      = in(14+NN);
    % we      = in(15+NN);
    % psi     = in(16+NN);
    state     =  in(1+NN:16+NN);
    NN = NN + 16;
    t         = in(1+NN);

    p = [pn; pe; -h];
    R = p_drone.R_min*1.3;

    persistent wpt_list_prev          % stored copy of old waypoints
    persistent idx                    % waypoint pointer
    persistent flag_transition        % state of transition state machine
    persistent flag_need_new_wpts     % flag that request new waypoints from path planner


    if start_of_simulation || isempty(wpt_list_prev)
      wpt_list_prev = zeros(5,p_drone.size_wpt_array);
      flag_need_new_wpts = 0;
    end

    % If the waypoints have changed, update the waypoint pointer
    if min( min(wpt_list==wpt_list_prev) )==0
      idx = 2;
      wpt_list_prev = wpt_list;
      flag_transition = 1;
      flag_need_new_wpts = 0;
    end

    % Define current and next two wpts
    wpt_prev = wpt_list(:,idx-1);
    wpt      = wpt_list(:,idx);
    wpt_next = wpt_list(:,idx+1);

    r_prev = wpt_prev(1:3);
    r      = wpt(1:3);
    r_next = wpt_next(1:3);

    q_prev = (r-r_prev) / norm(r-r_prev);
    q      = (r_next-r) / norm(r_next-r);
    rho_p    = acos(-(q_prev')*q);

    Va_d   = wpt(5);  % desired airspeed along waypoint path

    % Define transition state machine
    switch flag_transition
      case 1        % follow straight line from wpt_a to wpt_b
          flag   = 1;       % following straight line path
          r_next = r;
          r      = r_prev;
          q      = q_prev;
          z      = wpt(1:3) - (R/tan(rho_p/2))*q_prev;
          c      = r;       % not used for waypoint path
          rho    = R;       % not used for waypoint path
          lambda = 1;        % not used for waypoint path

          if (p-z)'*q_prev >= 0
              flag = 2;
              flag_transition = flag;
          end

      case 2        % follow orbit from wpt_a-wpt_b to wpt_b-wpt_c
          flag   = 2;  % following orbit
          c      = r - (R/(sin(rho_p/2)))* (q_prev-q)/norm(q_prev-q) ;
          rho    = R;
          lambda = sign(q_prev(1)*q(2) - q_prev(2)*q(1));
          z      = r + (R/tan(rho_p/2))*q;
          if (p-z)'*q >= 0
              if idx < p_drone.size_wpt_array -1
                    idx = idx + 1;
              end
              flag = 1;
              flag_transition = flag;
          end

    end

    out = [flag; Va_d; r; q; r_next; c; rho; lambda; state; flag_need_new_wpts];
end