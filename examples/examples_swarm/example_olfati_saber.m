%% Clear console and workspace and add project root to path

close all;
clearvars -except app;

project_root = strcat(extractBefore(mfilename('fullpath'),mfilename),'../..');
addpath(genpath(project_root));


%% Simulation options

DRONE_TYPE = "point_mass"; % swarming mode supports quadcopter and point_mass
ACTIVE_ENVIRONMENT = true;
DEBUG = true;
VIDEO = true;
CENTER_VIEW_ON_SWARM = false; 
SWARM_ALGORITHM = "olfati_saber"; % either vasarhelyi or olfati_saber

if DEBUG || VIDEO
    results_dirname = strcat('results/results_swarm');
    date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
    subfolder = strcat(erase(mfilename,"example_"), '_', date_string);
    results_dirname = strcat(results_dirname, '/', subfolder);
    if ~exist(results_dirname, 'dir')
        mkdir(results_dirname)
    end
end

fontsize = 12;


%% Get changes from GUI

if exist('app', 'var')
    % Simulation parameters
    p_sim.end_time = app.sim_time;
    
    % Drone parameters
    DRONE_TYPE = app.drone_type;
    
    % Swarming parameters
    SWARM_ALGORITHM = "olfati_saber";
    p_swarm.nb_agents = app.nb_agents;
    p_swarm.d_ref = app.d_ref;
    p_swarm.v_ref = app.v_ref;
    
    % Map parameters
    ACTIVE_ENVIRONMENT = app.active_environment;
    
    % Debug plot
    DEBUG = app.debug_plot;
end

if DRONE_TYPE == "point_mass"
   SWARM_VIEWER_TYPE = "agent";
elseif DRONE_TYPE == "quadcopter"
   SWARM_VIEWER_TYPE = "drone";
end


%% Call parameters files

run('param_sim');
run('param_battery');
run('param_physics');
if DRONE_TYPE == "fixed_wing" || DRONE_TYPE == "quadcopter"
    run('param_drone'); 
elseif DRONE_TYPE == "point_mass"
    run('param_drone'); 
end
run('param_map');
run('param_swarm');


%% Init Swarm object, Wind, Viewer and other variables

% Init swarm and set positions
swarm = Swarm();
swarm.algorithm = SWARM_ALGORITHM;

for i = 1 : p_swarm.nb_agents
    swarm.add_drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics,...
         map);
end
swarm.set_pos(p_swarm.Pos0);

% Init wind
wind = zeros(6,1); % steady wind (1:3), wind gusts (3:6)

% Init video
if VIDEO    
    video_filename = strcat(erase(mfilename, "example_"), '_', date_string);
    video_filepath = strcat(results_dirname, '/', video_filename);
    video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
end

% Init viewer
swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
swarm_viewer.viewer_type = SWARM_VIEWER_TYPE;
states_handle = [];

%% Main simulation loop

goal_reached = false;

disp('Type CTRL-C to exit');
for time = p_sim.start_time:p_sim.dt:p_sim.end_time
    
    % Check if program terminated from GUI
    if exist('app', 'var')
        switch app.StartsimulationSwitch.Value
            case 'Off'
                close all;
                return;
        end
    end
    
    % Get change from GUI
    if exist('app', 'var')
        % Wind parameters
        wind_active = app.wind;
        wind_gust_active = app.wind_gust;
        wind_level = app.wind_level;
        wind_gust_level = app.wind_gust_level;
        
        % Debug plot
        debug_plot = app.debug_plot;
        
        % Orientation of swarm migration
        orientation = app.orientation;
        p_swarm.u_ref = [-cosd(orientation), -sind(orientation), 0]';
    end
    
 
    % Compute velocity commands from swarming algorithm
    [~, collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);
    
    % Update swarm states and plot the drones
   swarm.update_state(wind, time);

   
   % Plot state variables for debugging
    if DEBUG
        swarm.plot_state(time, p_sim.end_time, ...
            1, p_sim.dt_plot, collisions, p_swarm.r_coll/2);
    end
    
    % Update video
    if VIDEO
        swarm_viewer.update(time, swarm, map);
        video.update(time, swarm_viewer.figure_handle);  
    end

end

if VIDEO
    video.close(); 
end

% Close all plots
close all;

if DEBUG && ~isempty(results_dirname)
    %% Plot offline viewer
    
    swarm_viewer_off = SwarmViewerOffline(p_sim.dt_video, ...
    CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map);


    %% Analyse swarm state variables
    
    time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
    pos_ned_history = swarm.get_pos_ned_history();
    pos_ned_history = pos_ned_history(2:end,:);
    vel_ned_history = swarm.get_vel_xyz_history();
    accel_history = [zeros(1, p_swarm.nb_agents*3); ...
        diff(vel_ned_history,1)/p_sim.dt];
    
    % Save workspace
    wokspace_path = strcat(results_dirname,'/state_var');
    save(wokspace_path,'time_history','pos_ned_history','vel_ned_history', ...
        'accel_history');
    
    % Plot state variables
    agents_color = swarm.get_colors();
    lines_color = [];

    plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
        accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
        results_dirname);

    
    %% Analyse performance
    
    % Compute swarm performance
    [safety, order, union, alg_conn, safety_obs, min_d_obs] = ...
        compute_swarm_performance(pos_ned_history, vel_ned_history, ...
        p_swarm, results_dirname);
    
    % Plot performance
    [perf_handle] = plot_swarm_performance(time_history', safety, order, ...
        union, alg_conn, safety_obs, min_d_obs, p_swarm, fontsize, results_dirname);
    
    
end


disp('Simulation completed successfully');
