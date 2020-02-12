function out = path_manager_dubins(in,p_drone,start_of_simulation)

% PATH_MANAGER_DUBINS - follow Dubins paths between waypoint configurations
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
%                  and Va_d is the desired airspeed along the path. 
%
% Outputs:
%   flag   - if flag==1, follow waypoint path
%            if flag==2, follow orbit
%   Va_d   - desired airspeed
%   r      - inertial position of start of waypoint path
%   q      - unit vector that defines inertial direction of waypoint path
%   c      - center of orbit
%   rho    - radius of orbit
%   lambda - direction of orbit (+1 for CW, -1 for CCW)
%

  NN = 0;
  nb_wpts = in(1+NN);
  wpt_list = reshape(in(2+NN:5*p_drone.size_waypoint_array+1+NN),5,p_drone.size_waypoint_array);
  NN = NN + 1 + 5*p_drone.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     = in(1+NN:16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 
  
  p = [pn; pe; -h];
  R = p_drone.radius;                 % radius of circular orbits

  persistent wpt_list_prev      % stored copy of old waypoints
  persistent idx_a              % waypoint pointer
  persistent flag_transition    % state of transition state machine
  persistent dubinspath
  persistent flag_need_new_wpts % flag that request new waypoints from path planner
  persistent flag_first_time_in_state
  
  if start_of_simulation
      wpt_list_prev = zeros(5,p_drone.size_wpt_array);
      flag_need_new_wpts = 0;
      flag_transition = 0;
      flag_first_time_in_state = 1;
  end
  
  
  % If waypoints have changed, update the waypoint pointer and plan new
  % Dubin's path
  if min(min(wpt_list==wpt_list_prev))==0
      wpt_list_prev = wpt_list;
      if start_of_simulation
        flag_transition = 0;
      end
      idx_a = 1;
      idx_b = 2;
      start_node = [wpt_list(1:4,idx_a)', 0, 0];
      end_node   = [wpt_list(1:4,idx_b)', 0, 0];      
      dubinspath = compute_dubins_param(start_node, end_node, R);  
      flag_need_new_wpts = 0;
      flag_first_time_in_state = 1;
  end
  
  % Define transition state machine
  switch flag_transition
      case 0             % beginning of simulation
          flag   = 1;
          Va_d   = p_drone.Va0;
          r      = dubinspath.ps;
          q      = dubinspath.q1;
          c      = dubinspath.cs;       % not used for waypoint path
          rho    = dubinspath.R;        % not used for waypoint path
          lambda = dubinspath.lams;     % not used for waypoint path
          if flag_first_time_in_state
              flag_first_time_in_state =0;
          end
          flag_transition = 1;
      
      case 1	% follow first orbit on Dubins path until intersect H1
          flag   = 2;
          Va_d   = p_drone.Va0;
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = dubinspath.cs;       % not used for waypoint path
          rho    = dubinspath.R;        % not used for waypoint path
          lambda = dubinspath.lams;     % not used for waypoint path
    
          if ((p-dubinspath.w1)'*dubinspath.q1 >= 0)&&(flag_first_time_in_state==1) % start in H1
              flag_transition = 2;
              flag_first_time_in_state = 1;
           elseif (p-dubinspath.w1)'*dubinspath.q1 >= 0 % entering H1
              flag_transition = 3;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
      case 2    % follow first orbit on Dubins path until on right side of H1
          flag   = 2;
          Va_d   = p_drone.Va0;
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = dubinspath.cs;       % not used for waypoint path
          rho    = dubinspath.R;        % not used for waypoint path
          lambda = dubinspath.lams;     % not used for waypoint path
          
          if (p-dubinspath.w1)'*dubinspath.q1 < 0 % get to right side H1
              flag_transition = 1;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
      case 3    % follow straight line on Dubins path until intersect H2
          flag   = 1;
          Va_d   = p_drone.Va0;
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = dubinspath.cs;       % not used for waypoint path
          rho    = dubinspath.R;        % not used for waypoint path
          lambda = dubinspath.lams;     % not used for waypoint path
          flag_first_time_in_state = 0;
          
          if (p-dubinspath.w2)'*dubinspath.q1 >= 0 % entering H2
              flag_transition = 4;
              flag_first_time_in_state = 1;
          end
              
      case 4    % follow second orbit on Dubins path until intersect H3
          flag   = 2;
          Va_d   = p_drone.Va0;
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = dubinspath.ce;       % not used for waypoint path
          rho    = dubinspath.R;        % not used for waypoint path
          lambda = dubinspath.lame;     % not used for waypoint path
          flag_first_time_in_state = 0;
          
          if ((p-dubinspath.w3)'*dubinspath.q3 >= 0)&&(flag_first_time_in_state==1) % start in H3
              flag_transition = 5;
              flag_first_time_in_state=1;
          elseif (p-dubinspath.w3)'*dubinspath.q3 >= 0 % entering H3
              % increase the waypoint pointer
              if idx_a==nb_wpts-1
                  flag_need_new_wpts = 1;
                  idx_b = idx_a+1;
              else
                  idx_a = idx_a+1;
                  idx_b = idx_a+1;
                  flag_transition = 1;
                  flag_first_time_in_state = 1;
              end
              % Plan new Dubin's path to next waypoint configuration
              start_node = [wpt_list(1:4,idx_a)', 0, 0];
              end_node   = [wpt_list(1:4,idx_b)', 0, 0];      
              dubinspath = compute_dubins_param(start_node, end_node, R);    
          else
              flag_first_time_in_state = 0;
          end

      case 5    % follow first orbit on Dubins path until on right side of H3
          flag   = 2;
          Va_d   = p_drone.Va0;
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = dubinspath.cs;       % not used for waypoint path
          rho    = dubinspath.R;        % not used for waypoint path
          lambda = dubinspath.lams;     % not used for waypoint path
          flag_first_time_in_state = 0;
          
          if (p-dubinspath.w1)'*dubinspath.q1 < 0 % get to right side of H3
              flag_transition = 4;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
              
  end
  
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_wpts];

end

