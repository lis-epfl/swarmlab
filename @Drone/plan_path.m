function plan_path(self, path_type, time)
% PLAN_PATH
%
% Inputs:
%   self - a drone
%   path_type
%   time
 
    flag_new_waypoints = self.path(end);
 
    p_drone        = self.p_drone;
    p_battery      = self.p_battery;
    p_sim          = self.p_sim;
    p_physics      = self.p_physics;
    map            = self.map;
    
    pn       = self.z_hat(1);
    pe       = self.z_hat(2);
    h        = self.z_hat(3);
    chi      = self.z_hat(9);
 
    nb_wpts  = self.nb_waypoints;
    wpt_list = self.waypoints;
    
    if (time==0) || flag_new_waypoints
        % format for each point is [pn, pe, pd, chi, Va_d] where the position
        % of the waypoint is (pn, pe, pd), the desired course at the waypoint
        % is chi, and the desired airspeed between waypoints is Va
        % if chi!=-9999, then Dubins paths will be used between waypoints.
 
        switch path_type
            case "fillet"  
                nb_wpts = 4;
                scale = 10;
                wpts = [0, 0, -10;
                        300, 0, -10;
                        0, 300, -10;
                        300, 300, -10];
                wpts_info = [-9999, p_drone.Va0;
                             -9999, p_drone.Va0;
                             -9999, p_drone.Va0;
                             -9999, p_drone.Va0];
                wpt_list = [scale*wpts, wpts_info];
 
            case "dubins"  % Dubins
                nb_wpts = 5;
                scale = 10;
                wpts = [0, 0, -7;
                        100, 0, -7;
                        100, 150, -7;
                        200, 150, -7;
                        200, 400, -7];
                wpts_info = [0, p_drone.Va0;
                             45*pi/180, p_drone.Va0;
                             0*pi/180, p_drone.Va0;
                             45*pi/180, p_drone.Va0;
                             0*pi/180, p_drone.Va0];
                wpt_list = [scale*wpts, wpts_info];
 
            case "city_dubins"  % Path through city using Dubin's paths
                % Current configuration
                wpt_start = [pn, pe, -h, chi, p_drone.Va0];
                % Desired end waypoint
                if norm([pn; pe; -h]-[map.width; map.width; -h])<map.width/2
                  wpt_end = [0, 0, -h, 0, p_drone.Va0];
                else
                  wpt_end = [map.width, map.width, p_drone.pd_T, 0, p_drone.Va_T];
                end
                waypoints = plan_RRT_dubins(wpt_start, wpt_end, p_drone.R_min, map);
                nb_wpts = size(waypoints,1);
                wpt_list = [];
                for i=1:nb_wpts
                    wpt_list = [wpt_list; ...
                        waypoints(i,-63), waypoints(i,2), waypoints(i,3), waypoints(i,4), p_drone.Va0];
                end
 
            case "straight_line_rrt"  % Path through city using straight-line RRT
                % Current configuration
                wpt_start = [pn, pe, -h, chi, p_drone.Va0];
                % Desired end waypoint
                if norm([pn; pe; -h]-[map.width; map.width; -h])<map.width/2
                  %wpt_end = [0, 0, -h, 0, p_drone.Va0];
                  wpt_end = [map.width, map.width, -h, 0, p_drone.Va0];
                else
                  wpt_end = [map.width, map.width, -h, 0, p_drone.Va0];
                end
                waypoints = plan_RRT(wpt_start, wpt_end, map);
                nb_wpts = size(waypoints,1);
                wpt_list = [];
                for i=1:nb_wpts
                    wpt_list = [wpt_list; ...
                        waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), p_drone.Va0];
                end
 
            case 5  % Cover path through city using Dubin's paths
                % Current configuration
                wpt_start = [pn, pe, -h, chi, p_drone.Va0];
                waypoints = plan_cover_RRT_dubins(wpt_start, p_drone.R_min, map);
                nb_wpts = size(waypoints,1);
                wpt_list = [];
                for i=1:nb_wpts
                    wpt_list = [wpt_list; ...
                        waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), p_drone.Va0];
                end
        end    
        
        % Fills the rest of the array
        for i = nb_wpts+1:p_drone.size_waypoint_array
              wpt_list = [wpt_list; ...
                     -9999, -9999, -9999, -9999, -9999];
        end
    end
    
self.nb_waypoints = nb_wpts; 
self.waypoints = reshape(wpt_list', 5*p_drone.size_waypoint_array, 1);
 
end
