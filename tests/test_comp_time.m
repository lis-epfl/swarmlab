%% Clear console and workspace and add project root to path

close all;

set(groot,'DefaultAxesFontSize',14)
set(groot,'DefaultLineLineWidth',1.5)

% project_root = strcat(extractBefore(mfilename('fullpath'),mfilename));
% addpath(genpath(project_root));

%% Initialization

end_time = 10; % p_sim to evaluate 
iterations = 1;
algo_array = ["vicsek", "olfati_saber"];
drone_array = ["point_mass", "quadcopter"];
nb_agents_array = [2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]; % nb_agents to evaluate  
ct = zeros(length(iterations), length(algo_array), length(drone_array), length(nb_agents_array));

%% Loop

disp('Computational time analysis started')

for selected_iter = iterations
    for selected_algo = 1:length(algo_array)
        for selected_drone = 1:length(drone_array)
            for selected_nb_agents = 1:length(nb_agents_array)
                SWARM_ALGORITHM = algo_array(selected_algo);
                DRONE_TYPE = drone_array(selected_drone);
                ct(selected_iter,selected_algo, selected_drone, selected_nb_agents) = example_ct(algo_array(selected_algo), ...
                    drone_array(selected_drone), nb_agents_array(selected_nb_agents), end_time);
            end
        end
    end
end

%% Plotting

figure;

selected_iter = 1;
for selected_algo = 1:length(algo_array)
    for  selected_drone = 1:length(drone_array)
            h = plot(nb_agents_array, ct(selected_iter,selected_algo, selected_drone, :)/end_time,'-o');
            set(h, 'markerfacecolor', get(h, 'color'));
            hold on
    end
end
hold off
set(gca,'xscale','log')
set(gca,'yscale','log')
xlabel('Number of drones')
ylabel('Real-time factor')
xticks(nb_agents_array);
yticks([0 0.1 1 10]);
legend('vasarhelyi, point-mass', 'vasarhelyi, quadcopter', ...
        'olfati-saber, point-mass', 'olfati-saber, quadcopter', ...
        'Location','northwest')
   
%% End of simulation

disp('Computational time analysis completed successfully')