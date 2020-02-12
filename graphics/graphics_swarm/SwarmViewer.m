classdef SwarmViewer < handle
    % SWARMVIEWER: class to visualize the swarm in the ENU (East-North-Up)
    % 3D-space
    
    properties
        figure_handle
        plot_initialized
        output_rate
        time_of_last_frame
        center_view_on_swarm
        viewer_type SwarmViewerType
        scatter_handle
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = SwarmViewer(output_rate, center_view_on_swarm)
            self.plot_initialized = 0;
            self.output_rate = output_rate;
            self.time_of_last_frame = 0;
            self.center_view_on_swarm = center_view_on_swarm;
            self.scatter_handle = [];
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = update(self, time, swarm, map)
            
            if (time-self.time_of_last_frame) >= self.output_rate - 1e-6
                self.time_of_last_frame = time;
                
                %----------------------------------------------------------
                if self.plot_initialized == 0 % Init plot
                    x0 = 66; 
                    y0 = 1; 
                    width = 884;
                    height = 973;
                    self.figure_handle = figure('Name','Swarm viewer', ...
                        'NumberTitle','off', ...
                        'position', [x0, y0, width, height]); clf;
                    grid on;
                    
                    if map.ACTIVE_ENVIRONMENT
                        if strcmp(map.building_shape, 'parallelepiped')
                            draw_buildings(map);
                        elseif strcmp(map.building_shape, 'cylinder')
                            self.figure_handle = draw_cylinders(self.figure_handle, map);
                        end
                    end
                    
                    if self.viewer_type == "drone" % Draw drones' body
                        
                        for i = 1: swarm.nb_agents
                            
                            drone_type = swarm.drones(i).drone_type;
                            [vertices, faces, face_colors] = define_body(self, drone_type);
                            
                            % Set new graphic characteristics as output
                            swarm.drones(i).vertices = vertices;
                            swarm.drones(i).faces = faces;
                            swarm.drones(i).face_colors = face_colors;
                            
                            drone = swarm.drones(i);
                            self.draw_body(drone);
                        end
                        
                    elseif self.viewer_type == "agent" % Draw agents
                        
                        size_agents = repmat(900, swarm.nb_agents, 1);
                        colors = swarm.get_colors();
                        
                        pos_ned = swarm.get_pos_ned();
                        pn = pos_ned(1,:);
                        pe = pos_ned(2,:);
                        pd = pos_ned(3,:);
                        
                        self.scatter_handle = scatter3(pe, pn, -pd, size_agents, 'CData', colors', 'Marker', '.');
                        
                    else % Draw agents with energy                     
                    end
                    
                    % // TODO: Draw paths
                    
                    if self.center_view_on_swarm || ~map.ACTIVE_ENVIRONMENT
                        axis_lim = define_axis_lim(self, swarm);
                        axis_min_inverted = [axis_lim(2,:), ...
                            axis_lim(1,:), -axis_lim(3,2), -axis_lim(3,1)];
                        axis(axis_min_inverted(:));
                        view(self.figure_handle.CurrentAxes,[-10,-10,20]);
                    else
                        view(self.figure_handle.CurrentAxes,0,90);
                    end
                    axis square;
                    drawnow
                    
                    xlabel('Y position [m]');
                    ylabel('X position [m]');
                    zlabel('Z position [m]');
                    grid on;
                    self.plot_initialized = 1;
                    
                    set(self.figure_handle,'Units','pixels');
                    %                     axis_lim = self.define_axis_lim(swarm);
                    %                     xlim(axis_lim(1, 1:2));
                    %                     ylim(axis_lim(2, 1:2));
                    
                    %----------------------------------------------------------
                else % At every other time step, redraw
                    
                    if self.viewer_type == "drone" % Draw drones' body
                        for i = 1:swarm.nb_agents
                            drone = swarm.drones(i);
                            self.draw_body(drone);
                        end
                        
                    elseif self.viewer_type == "agent" % Draw agents
                        size_agents = repmat(900, swarm.nb_agents, 1);
                        colors = swarm.get_colors();
                        
                        pos_ned = swarm.get_pos_ned();
                        pn = pos_ned(1,:);
                        pe = pos_ned(2,:);
                        pd = pos_ned(3,:);
                        set(self.scatter_handle, 'Xdata', pe, 'Ydata', pn, ...
                            'Zdata', -pd, 'Marker', '.', 'SizeData', ...
                            size_agents, 'CData', colors');
                        
                    else % Draw agents with energy
                        
                    end
                    
                    if self.center_view_on_swarm || ~map.ACTIVE_ENVIRONMENT
                        axis_lim = define_axis_lim(self, swarm);
                        axis_min_inverted = [axis_lim(2,:), ...
                            axis_lim(1,:), -axis_lim(3,2), -axis_lim(3,1)];
                        axis(axis_min_inverted(:));
                        view(self.figure_handle.CurrentAxes,[-10,-10,20]);
                    end
                    drawnow;
                end
                title(self.figure_handle.CurrentAxes, sprintf('Simulation time: %.1f seconds', time));
                
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [vertices, faces, face_colors] = define_body(self, drone_type)
            
            if drone_type == "fixed_wing"
                [vertices, faces, face_colors] = define_wing_body();
                
            elseif drone_type == "quadcopter"
                [vertices, faces, face_colors] = define_quad_body();
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
                view([-10,-10,20]);
            else
                set(drone.body_handle,'Vertices',vertices',...
                    'Faces',drone.faces);
            end
        end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function axis_lim = define_axis_lim(self, swarm)
            % define_axis_lim - Returns the limits of the plot in the form
            % of a 3x2 matrix as follows:
            %
            % x_min x_max
            % y_min y_max
            % z_min z_max
            
            margin = 40;
            positions = swarm.get_pos_ned();
            min_pos = min(positions,[],2);
            max_pos = max(positions,[],2);
            edge = max(max_pos-min_pos);
            center = (min_pos + max_pos)/2;
            axis_lim = repmat(center,1,2) + ...
                repmat([-edge/2-margin +edge/2+margin], 3, 1);
        end
        
    end
end