classdef SwarmViewerOffline < handle
    % SwarmViewerOffline: class to visualize the swarm in the ENU (East-North-Up)
    % 3D-space
    
    properties
        figure_handle
        plot_initialized
        output_rate
        time_of_last_frame
        center_view_on_swarm
        scatter_handle
        lines_handle
        wakes 
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    methods
        %%%%%%%%%% Constructor %%%%%%%%%%%%
        function self = SwarmViewerOffline(output_rate, CENTER_VIEW_ON_SWARM, dt, swarm, map)
            
            wake_length = 300;
            pos_ned_history = swarm.get_pos_ned_history();
            
            % Instance
            self.plot_initialized = 0;
            self.output_rate = output_rate;
            self.time_of_last_frame = 0;
            self.center_view_on_swarm = CENTER_VIEW_ON_SWARM;
            self.scatter_handle = [];
            self.lines_handle = [];
            self.wakes = repmat(pos_ned_history(1,:),wake_length,1);
            
            % Update
            [nb_steps,~] = size(pos_ned_history);
            for step = 1:nb_steps
                pos_current = pos_ned_history(step, :);
                time = dt * step;
                self.update(swarm, time, pos_current, map, step);
            end
            
            % Close
            close(self.figure_handle);
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function self = update(self, swarm, time, pos_current, map, step)
            
            if (time-self.time_of_last_frame) >= self.output_rate
                self.time_of_last_frame = time;
                
                %----------------------------------------------------------
                if self.plot_initialized == 0 % Init plot
                    x0 = 0; 
                    y0 = 0; 
                    width = 1920;
                    height = 1080;
                    self.figure_handle = figure('Name','Swarm viewer offline',...
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
                    
                    colours = swarm.get_colors();
                    size_agents = repmat(30, swarm.nb_agents, 1); % size_agents is size of dot drawn in scatter3.
                    
                    pos_current_matrix = reshape(pos_current,3,[]);
                    pn = pos_current_matrix(1,:);
                    pe = pos_current_matrix(2,:);
                    pd = pos_current_matrix(3,:);

                    self.scatter_handle = scatter3(pe, pn, -pd, size_agents, colours', 'filled');
                    
                    % Draw wakes
                    for agent = 1:swarm.nb_agents
                        pn = self.wakes(:,3*(agent-1)+1);
                        pe = self.wakes(:,3*(agent-1)+2);
                        pd = self.wakes(:,3*(agent-1)+3);
                        hold on;
                        self.lines_handle(agent) = plot3(...
                            pe, pn, -pd, 'Color', colours(:,agent)');
                    end
                    
                    if self.center_view_on_swarm
                        axis_lim = define_axis_lim(self, swarm, pos_current_matrix);
                        axis_min_inverted = [axis_lim(2,:), ...
                            axis_lim(1,:), -axis_lim(3,2), -axis_lim(3,1)];
                        axis(axis_min_inverted(:));
                        view([-20,90,20]);
                    else
                        view(0,90);
                    end
                    
                    drawnow;
                    
                    xlabel('Y position [m]');
                    ylabel('X position [m]');
                    zlabel('Z position [m]');
                    axis square;
                    grid on;
                    hold on;                   
                    self.plot_initialized = 1;
                    %----------------------------------------------------------
                else % At every other time step, redraw
                    
                    size_agents = repmat(30, swarm.nb_agents, 1);
                    colours = swarm.get_colors();
                    
                    pos_current_matrix = reshape(pos_current,3,[]);
                    pn = pos_current_matrix(1,:);
                    pe = pos_current_matrix(2,:);
                    pd = pos_current_matrix(3,:);
                    
                    % set(self.scatter_handle, 'Xdata', pe, 'Ydata', pn, ...
                    %     'Zdata', -pd, 'Marker', '.', 'SizeData', ...
                    %     size_agents, 'CData', colours');
                    set(self.scatter_handle, 'Xdata', pe, 'Ydata', pn, ...
                        'Zdata', -pd);
                    xlabel('Y position [m]');
                    ylabel('X position [m]');
                    zlabel('Z position [m]');
                    axis square;
                    grid on;
                    hold on;
                    
                    % Draw wakes
                    for agent = 1:swarm.nb_agents
                        pn = self.wakes(:,3*(agent-1)+1);
                        pe = self.wakes(:,3*(agent-1)+2);
                        pd = self.wakes(:,3*(agent-1)+3);
                        hold on;
                        set(self.lines_handle(agent), 'Xdata', pe, 'Ydata', pn, ...
                        'Zdata', -pd) %, 'CData', colours(:,agent)');
                        
                    end
                    
                    if self.center_view_on_swarm
                        axis_lim = define_axis_lim(self, swarm, pos_current_matrix);
                        axis_min_inverted = [axis_lim(2,:), ...
                            axis_lim(1,:), -axis_lim(3,2), -axis_lim(3,1)];
                        axis(axis_min_inverted(:));
                        view([-10,-10,20]);
                    end
                    
                end
                
                signal_collision(self, swarm, step);
                drawnow;
                self.wakes = circshift(self.wakes, 1);
                self.wakes(1,:) = pos_current;
                title(sprintf('Simulation time: %.2f seconds', time));
            end
        end
        
        function axis_lim = define_axis_lim(self, swarm, current_pos_matrix)
            % define_axis_lim - Returns the limits of the plot in the form 
            % of a 3x2 matrix as follows:
            %
            % x_min x_max
            % y_min y_max
            % z_min z_max
            
            margin = 10;
            positions = current_pos_matrix;
            min_pos = min(positions,[],2);
            max_pos = max(positions,[],2);
            edge = max(max_pos-min_pos);
            center = (min_pos + max_pos)/2;
            axis_lim = repmat(center,1,2) + ...
            repmat([-edge/2-margin +edge/2+margin], 3, 1);
        end 
        
        function signal_collision(self, swarm, step)
            % Signals when drone-obstacle collisions take place. 
            %n = length(swarm.collisions_history);
            compare = 0;
            if ~isequal(swarm.collisions_history(step, 2), compare)
                self.figure_handle.Color = [1 0.8 0.8]; % when a collision takes place, set background to red
            end
            
        end
        
    end
end