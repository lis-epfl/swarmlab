function plot_swarm(time, end_time, X, X_opt_vector, cylinders_vector)

% PLOT_SWARM - generates a plot of the swarm. The plot is refreshed at each
% timestamp.
%
% Inputs:
%       t :             current time in the simulation
%       X :             positions and velocities of the agents
%       X_opt :         predicted positions and velocities of the agents
%

% Persistent variables
persistent swarm_fig
persistent scatter_handle
persistent colors
persistent prediction_handle
persistent video_handle

% Reformat inputs
N = length(X)/6;
X_mat = reshape(X,6,[]);
Pos = X_mat(1:3,:);
Pn = Pos(1, :)';
Pe = Pos(2, :)';
Pu = Pos(3, :)';
X_opt = reshape(X_opt_vector,[],length(X));
Pos_opt = X_opt(:,repmat([true true true false false false],1,N));

% Define plot limits
% axis_lim    = define_lim_plot(Pos);
% x_lim       = axis_lim(1, :);
% y_lim       = axis_lim(2, :);
% z_lim       = axis_lim(3, :);

% Parameters
s = repmat(300, N, 1); % marker size
my_blue = [0 0 1]';
video_filename = [];

% Initialize figure at t = 0
if isempty(swarm_fig)
    
    % Create figure handle
    swarm_fig = figure;
    clf(swarm_fig, 'reset');
    
    % Draw agents
    colors = repmat(my_blue, 1, N);
    scatter_handle = scatter3(Pe, Pn, Pu, s, colors', '.');
    hold(scatter_handle.Parent,'on');
    % set(scatter_handle.Parent, 'XLim', x_lim, 'YLim', y_lim, 'ZLim', z_lim);
    
    % Draw cylinders
    nb_edges  = 8;
    gray_shade = 0.8; % from 0 to 1
    if length(cylinders_vector)>1
        cylinders = reshape(cylinders_vector,3,[]);
        cyl_radius = cylinders(3,1);
        [X,Y,Z] = cylinder(cyl_radius,nb_edges);
        Z = 40 * Z;
        [~,nb_cylinders] = size(cylinders);
        if nb_cylinders > 0
            for i = 1:nb_cylinders
                Xtrasl = X + cylinders(1,i);
                Ytrasl = Y + cylinders(2,i);
                C = gray_shade*repmat(ones(size(Xtrasl)),1,1,3);
                surf( Ytrasl, Xtrasl, Z, C);
                hold on;
            end
        end
    end
    view(0,90);
    xlabel('Y Position [m]');
    ylabel('X Position [m]');
    zlabel('Z Position [m]');
    
    % Draw predicted path for every agent
    for agent = 1:N
        Pn_opt = Pos_opt(:,3*(agent-1)+1);
        Pe_opt = Pos_opt(:,3*(agent-1)+2);
        Pu_opt = Pos_opt(:,3*(agent-1)+3);
        prediction_handle(agent) = plot3(Pe_opt,Pn_opt,Pu_opt,'b');
    end
    axis('equal');
    
    % Open video
    if ~isempty(video_filename)
        video_handle = VideoWriter(video_filename);
        open(video_handle);
    end
    
else  % for t > 0
    
    set(scatter_handle, 'Xdata', Pe, 'Ydata', Pn, 'Zdata', Pu, ...
        'Marker', '.', 'SizeData', s, 'CData', colors');
    % set(scatter_handle.Parent, 'XLim', x_lim, 'YLim', y_lim, 'ZLim', z_lim);
    delete(prediction_handle);
    for agent = 1:N
        Pn_opt = Pos_opt(:,3*(agent-1)+1);
        Pe_opt = Pos_opt(:,3*(agent-1)+2);
        Pu_opt = Pos_opt(:,3*(agent-1)+3);
        hold(scatter_handle.Parent,'on');
        prediction_handle(agent) = plot3(Pe_opt,Pn_opt,Pu_opt,'b');
    end
    
    % Write frame on video
    if ~isempty(video_filename)
        frame = getframe(swarm_fig);
        writeVideo(video_handle, frame);

        % Close video
        if time == end_time
            close(video_handle);
            close(swarm_fig);
        end
    end
    
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function limits = define_lim_plot(points)
% DEFINE_LIM_PLOT - This function defines the axes limits for the plots,
% computed from a matrix of points.
%
% Inputs:
%   points   - matrix of points [X Y Z]'. Works also for [X Y]'.
%
% Outputs:
%   limits   - limits for the axis [x_min x_max; y_min y_max; z_min z_max]

max_pos     = max(points,[],2);
min_pos     = min(points,[],2);
avg_pos     = mean(points,2);
edge_len    = 1.1 * max(max_pos - min_pos);
if edge_len == 0
    edge_len = 10;
end
limits      = [avg_pos-0.5*edge_len, avg_pos+0.5*edge_len];

end
