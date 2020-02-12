function computational_time = example_ct(SWARM_ALGORITHM, DRONE_TYPE, nb_agents, end_time)

    t_start = tic;

    %% Running example_vicsek.m

    ACTIVE_ENVIRONMENT = false;
    DEBUG = false;
    VIDEO = false;
    CENTER_VIEW_ON_SWARM = false;
    SWARM_VIEWER_TYPE = "agent"; % drone, agent or agent_with_energy 
    
    if SWARM_VIEWER_TYPE == "" 
       SWARM_VIEWER_TYPE = (input(strcat('Select a swarm type viewer among the following: \n',...
                        'drone, \n', ...
                        'agent \n', ...
                        'agent_with_energy'),'s'));
    end

    fontsize = 12;

    %% Call parameter files
    
    p_sim.end_time = end_time;
    run('param_sim');
    run('param_battery');
    run('param_physics');
    if DRONE_TYPE == "fixed_wing" || DRONE_TYPE == "quadcopter"
        run('param_drone'); 
    elseif DRONE_TYPE == "point_mass"
        run('param_drone'); 
    end
    run('param_map'); % creates map: struct for the city parameters
    p_swarm.nb_agents = nb_agents;
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
    wind = zeros(6,1);

    % Init variables for history
%     x0 = [p_swarm.Pos0; zeros(3,p_swarm.nb_agents)];
%     x_history(1,:) = x0(:);

    % Init video
    if VIDEO
        % Init viewer
        swarm_viewer = SwarmViewer(p_sim.dt_plot, CENTER_VIEW_ON_SWARM);
        swarm_viewer.viewer_type = SWARM_VIEWER_TYPE;
        states_handle = [];
        video_folder = 'videos/videos_swarm/';
        if ~exist(video_folder, 'dir')
           mkdir(video_folder);
        end
        video_filename = strcat(video_folder, mfilename);
        video = VideoWriterWithRate(video_filename, p_sim.dt_video);
    end

    %% Main simulation loop

    goal_reached = false;

    disp('Type CTRL-C to exit');
    for time = p_sim.start_time:p_sim.dt:p_sim.end_time

        % Compute wind
    %     for i = 1 : p_swarm.nb_agents
    %         wind = get_wind(time, map, p_sim.dt, p_sim.end_time, swarm.drones(i).pos_ned, ...
    %             wind_active, wind_gust_active, wind_level, wind_gust_level);
    %         swarm.drones(i).wind = wind; 
    %     end

        % Compute velocity commands from swarming algorithm
        [vel_c,collisions] = swarm.update_command(p_swarm, p_swarm.r_coll, p_sim.dt);

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

    %% Plot offline viewer
%     swarm_viewer_off = SwarmViewerOffline(p_sim.dt_video, ...
%         CENTER_VIEW_ON_SWARM, p_sim.dt, swarm, map);

    if DEBUG && ~isempty(debug_dirname)
        %% Analyse swarm state variables

        time_history = p_sim.start_time:p_sim.dt:p_sim.end_time;
        pos_ned_history = swarm.get_pos_ned_history();
        pos_ned_history = pos_ned_history(3:end,:);
        vel_ned_history = swarm.get_vel_xyz_history();
        vel_ned_history = vel_ned_history(2:end,:);
        accel_history = [zeros(1, p_swarm.nb_agents*3); ...
            diff(vel_ned_history,1)/p_sim.dt];

        % Save workspace
        wokspace_path = strcat(debug_dirname,'/workspace');
        save(wokspace_path);

        % Plot state variables
        agents_color = swarm.get_colors();
        lines_color = [];

        plot_state_offline(time_history', pos_ned_history, vel_ned_history, ...
            accel_history, agents_color, p_swarm, map, fontsize, lines_color, ...
            debug_dirname);

        %% Analyse performance

        % Compute swarm performance
        [safety, order, union, alg_conn, safety_obs] = ...
            compute_swarm_performance(pos_ned_history, vel_ned_history, ...
            p_swarm, debug_dirname);

        % Plot performance
        [perf_handle] = plot_swarm_performance(time_history', safety, order, ...
            union, alg_conn, safety_obs, p_swarm, fontsize, debug_dirname);
    end
        %% Computational time calculation
        computational_time = toc(t_start);

end

