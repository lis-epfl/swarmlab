function out = path_manager_wing(in,p_drone, p_sim)

% PATH_MANAGER_WING
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
%   c       - center of orbit
%   rho     - radius of orbit
%   lambda  - direction of orbit (+1 for CW, -1 for CCW)
%   state   - drone state
%   flag_new_wpt  - demand new waypoints from path planer
%

    persistent start_of_simulation

    t = in(end);
    if t==0 || t==p_sim.dt
      start_of_simulation = 1;
    end

    NN = 0;
    nb_wpts = in(1+NN);
    if nb_wpts==0         % start of simulation
        flag   = 1;       % following straight line path
        Va_d   = p_drone.Va0;   % desired airspeed along waypoint path
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
        out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_wpts];
    else
        wpt_list = reshape(in(2+NN:5*p_drone.size_wpt_array+1+NN),5,p_drone.size_wpt_array);

        if abs(wpt_list(4,1))>=2*pi
            out = path_manager_fillet(in,p_drone,start_of_simulation);  % smooths through waypoints with fillets
            start_of_simulation=0;
        else
            out = path_manager_dubins(in,p_drone,start_of_simulation);  % follows Dubins paths between waypoint configurations
            start_of_simulation=0;
        end
  end

end