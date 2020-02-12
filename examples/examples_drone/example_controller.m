%% Clear console and workspace and add project root to path

close all;
clearvars -except app;

project_root = strcat(extractBefore(mfilename('fullpath'),mfilename),'../..');
addpath(genpath(project_root));

%% Simulation options

DRONE_TYPE = "quadcopter"; % either fixed-wing, or quadcopter
DEBUG = true; % true to plot the state of the drone, false otherwise
VIDEO = true; % true to record a video, false otherwise
CENTER_VIEW_ON_DRONE = true; % true to center the drone in the viewer, false otherwise
WIND_ACTIVE = false; % true to activate steady wind, false otherwise
WIND_GUST_ACTIVE = false; % true to activate wind gusts, false otherwise

%% Run parameters files

run('param_physics');
run('param_drone'); 
run('param_sim');
run('param_battery');
run('param_map'); 

%% Initalize wind

wind_level = 0;
wind_gust_level = 0;
wind = zeros(6, 1); % steady wind (1:3), wind gusts (3:6)

%% Get changes from GUI

if exist('app', 'var')
    % Simulation parameters
    p_sim.end_time = app.sim_time;
    
    % Drone parameters
    DRONE_TYPE = app.drone_type;
    
    % Debug plot
    DEBUG = app.debug_plot;
end

%% Initialize drone, video writer and drone viewer

% Init drone
drone = Drone(DRONE_TYPE, p_drone, p_battery, p_sim, p_physics, map);
drone.set_pos([rand() * map.width; rand() * map.width; -rand() * map.max_height]);

% Init video writer
if VIDEO
    results_folder = 'results/results_drone/';
    if ~exist(results_folder, 'dir')
       mkdir(results_folder);
    end
    date_string = datestr(now,'yyyy_mm_dd_HH_MM_SS');
    video_filename = strcat(mfilename, '_', date_string);
    video_filepath = strcat(results_folder, video_filename);
    video = VideoWriterWithRate(video_filepath, p_sim.dt_video);
end

% Init drone viewer
drone_viewer = DroneViewer(drone, p_sim.dt_plot, CENTER_VIEW_ON_DRONE);

%% Main simulation loop

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
    
    % Get wind
    if exist('app', 'var')
        % Wind parameters
        WIND_ACTIVE = app.wind;
        WIND_GUST_ACTIVE = app.wind_gust;
        wind_level = app.wind_level;
        wind_gust_level = app.wind_gust_level;
        
        % Debug plot
        DEBUG = app.debug_plot;
    end
    
    % Create wind
    wind = get_wind(time, map, p_sim.dt, p_sim.end_time, drone.pos_ned, WIND_ACTIVE, ...
        WIND_GUST_ACTIVE, wind_level, wind_gust_level);
    
    % Create command
    drone.command = generate_command(drone.drone_type, drone.autopilot_version, time);
    
    % Update drone states and plot
    drone.update_state(wind, time);
    
    % Update drone viewer
    drone_viewer.update(time, drone, []);
    
    % Update plot of state variables (for debugging)
    if DEBUG
        drone.plot_state(time, p_sim.dt_plot);
    end
    
    % Update video
    if VIDEO
        video.update(time, drone_viewer.figure_handle);
    end

end

if VIDEO
    video.close();
end

disp('Simulation completed successfully');