%% Clear console and workspace and add project root to path
clear all
close all

set(groot,'DefaultAxesFontSize',14)
set(groot,'DefaultLineLineWidth',1.5)

%% Initialization

end_time = 500; % p_sim to evaluate 
algo_array = ["vicsek", "olfati_saber"];
neig_array = [2, 4, 8, 16, 32, 64];
nb_agents = 64; 
co_mn = zeros(3, length(algo_array), length(neig_array));

%% Loop

disp('Collision analysis started')

for select_algo = 1:length(algo_array)
    for select_neig = 1:length(neig_array)
        SWARM_ALGORITHM = algo_array(select_algo);
        max_neig = neig_array(select_neig);
        co_mn(:, select_algo, select_neig) = example_co_mn(algo_array(select_algo), ...
            neig_array(select_neig), nb_agents, end_time);
    end
end

%% Plotting

figure

for select_algo = 1:length(algo_array)
    %for  select_neig = 1:length(neig_array)
            scatter(neig_array,  abs(co_mn(2, select_algo, :)), 900, '.');
            hold on
    %end
end

hold off
set(gca,'yscale','log')
title('Collision analysis: agent-obstacle')
xlabel('Number of max neighbours')
ylabel('Collisions')
grid on
legend('vicsek', 'olfati-saber', 'Location', 'northwest')    

figure

for select_algo = 1:length(algo_array)
    %for  select_neig = 1:length(neig_array)
            scatter(neig_array, abs(co_mn(1, select_algo, :)), 900, '.');
            hold on
    %end
end

hold off
set(gca,'yscale','log')
title('Collision analysis: agent-agent')
xlabel('Number of max neighbours')
ylabel('Collisions')
grid on
legend('vicsek', 'olfati-saber', 'Location', 'northwest')   
%%

disp('Computational time analysis completed successfully')