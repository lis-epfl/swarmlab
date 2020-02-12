classdef DroneViewer < handle
    % DroneViewer: class to visualize the drone in the ENU 3D-space

    properties
        figure_handle
        plot_initialized
        output_rate
        time_of_last_frame
        center_view_on_drone
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = DroneViewer(drone, output_rate, ...
                center_view_on_drone)
            [drone.vertices, drone.faces, drone.face_colors] = ...
                self.define_body(drone.drone_type);
            self.plot_initialized = 0;      
            self.output_rate = output_rate;
            self.time_of_last_frame = 0;
            self.center_view_on_drone = center_view_on_drone;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = update(self, time, drone, map)
            
            if (time-self.time_of_last_frame) >= self.output_rate
                
                self.time_of_last_frame = time;
                
                if self.plot_initialized == 0 % Init plot
                    x0 = 66; 
                    y0 = 1; 
                    width = 884;
                    height = 973;
                    self.figure_handle = figure('Name','Drone viewer', ...
                        'NumberTitle','off', ...
                        'position', [x0, y0, width, height]); clf;
                    
                    if ~isempty(map) && map.ACTIVE_ENVIRONMENT
                        draw_buildings(map);
                    end
                    
                    if drone.drone_type ~= "point_mass"
                        self.draw_body(drone);
                    else
                        self.draw_point(drone);
                    end

                    if ~isempty(map) && map.ACTIVE_ENVIRONMENT
                        path = drone.path;
                        nb_wpts = drone.nb_waypoints;
                        wpt_list = reshape(drone.waypoints(1:5*nb_wpts), 5, [])';
                        radius = path(12);
                        map_width = map.width;
                        %[drone.waypoint_handle, drone.body_handle] = draw_waypoints(self.figure_handle, wpt_list, radius, [], 'normal');
                        drone.waypoint_handle = draw_waypoints(self.figure_handle, wpt_list, radius, [], 'normal');
                        drone.path_handle = draw_path(self.figure_handle, path, map_width, []);
                    end
                    
                    xlabel('y position [m]');
                    ylabel('x position [m]');
                    zlabel('z position [m]');
                    grid(self.figure_handle.CurrentAxes);
                    self.plot_initialized = 1;
                    
                else % At every other time step, redraw
                    % Draw drone body
                    if drone.drone_type ~= "point_mass"
                        self.draw_body(drone);
                    else
                        self.draw_point(drone);
                    end

                    if ~isempty(map)
                        % Draw path
                        path = drone.path;
                        nb_wpts = drone.nb_waypoints;
                        wpt_list = reshape(drone.waypoints(1:5*nb_wpts), 5, [])';
                        if drone.drone_type == "quadcopter" || ...
                                drone.drone_type == "point_mass" % force only lines
                            radius = 0;
                        elseif drone.drone_type == "fixed_wing" % possible to have curves
                            radius = path(12);
                        end
                        map_width = map.width;
                        draw_waypoints(self.figure_handle, wpt_list, radius, drone.waypoint_handle);
                        draw_path(self.figure_handle, path, map_width, drone.path_handle);
                    end
                end
                title(self.figure_handle.CurrentAxes, sprintf('Simulation time: %.2f seconds', time));
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [vertices, faces, face_colors] = define_body(self, drone_type)
            
            if drone_type == "fixed_wing"
                [vertices, faces, face_colors] = define_wing_body();

            elseif drone_type == "quadcopter"
                [vertices, faces, face_colors] = define_quad_body();
                
            elseif drone_type == "point_mass"
                vertices = [];
                faces = [];
                face_colors = [];
            end 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = draw_body(self, drone)
            
            % inertial North/East/Down positions [m]
            pn = drone.pos_ned(1);
            pe = drone.pos_ned(2);
            pd = drone.pos_ned(3);
            % roll/pitch/yaw angles [rad]
            phi = drone.attitude(1);
            theta = drone.attitude(2);
            psi = drone.attitude(3);
            
            
            
            % Rotate rigid body
            vertices = rotate(drone.vertices, phi, theta, psi);
            % Translate after rotation
            vertices = translate(vertices, pn, pe, pd); 
            % Transform vertices from NED to ENU (for Matlab rendering)
            R = [...
                0, 1, 0;...
                1, 0, 0;...
                0, 0, -1;...
                ];
            vertices = R * vertices;
            
            if isempty(drone.body_handle)
                drone.body_handle = patch('Vertices', vertices', ...
                    'Faces', drone.faces,...
                    'FaceVertexCData',drone.face_colors,...
                    'FaceColor','flat');
                axis square;
                view(32, 47);
            else
                set(drone.body_handle,'Vertices',vertices',...
                    'Faces',drone.faces);
            end
            
            if self.center_view_on_drone == true
                a = self.figure_handle;
                margin = 10;
                a.CurrentAxes.XLim = [pe - margin, pe + margin];
                a.CurrentAxes.YLim = [pn - margin, pn + margin];
                a.CurrentAxes.ZLim = [-pd - margin, -pd + margin];
            end
            drawnow
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = draw_point(self, drone)
            
            % inertial North/East/Down positions [m]
            pn = drone.pos_ned(1);
            pe = drone.pos_ned(2);
            pd = drone.pos_ned(3);
            
            if isempty(drone.body_handle)
                drone.body_handle = scatter3(pe, pn, -pd);
                axis square;
                view(32, 47);
            else
                scatter3(drone.body_handle,pe, pn, -pd);
            end
            
            if self.center_view_on_drone == true
                a = self.figure_handle;
                margin = 10;
                a.CurrentAxes.XLim = [pe - margin, pe + margin];
                a.CurrentAxes.YLim = [pn - margin, pn + margin];
                a.CurrentAxes.ZLim = [-pd - margin, -pd + margin];
            end
            drawnow
        end
        
    end
    
end