function [dist_handle] = plot_distance_offline(pos_history, ...
    time_history, r_coll, d, r_comm, max_neig, fontsize, color)

% PLOT_DISTANCE_OFFLINE - Plot the min/avg/max distance of all the pairs of
% agents

[t_steps,~] = size(pos_history);
dist_handle = figure('Name','Offline swarm distances','NumberTitle','off');
min_dist = zeros(t_steps,1);
max_dist = zeros(t_steps,1);
avg_dist = zeros(t_steps,1);

for k = 1:t_steps
    Pos_k = pos_history(k,:);
    Pos_k = reshape(Pos_k,3,[]);
    Dist_k_matrix = pos2dist(Pos_k');
    M = compute_neighborhood(Dist_k_matrix, r_comm, max_neig);
    dist_neig = Dist_k_matrix(M==true);
    if ~isempty(dist_neig)
        % Distance of all couples of neighboring agents (counted twice if they
        % see each other)
        min_dist(k) = min(dist_neig);
        max_dist(k) = max(dist_neig);
        avg_dist(k) = mean(dist_neig);
    else
        min_dist(k) = NaN;
        max_dist(k) = NaN;
        avg_dist(k) = NaN;
    end
end

err_bar(1,:,:) = [max_dist-avg_dist, avg_dist-min_dist];
if ~isempty(color)
    line_props.col = {color};
    mseb(time_history',avg_dist',err_bar,line_props);
else
    mseb(time_history',avg_dist',err_bar);
end
hold on;
reference = yline(d,'--','LineWidth',1.5);
reference.Color = [0.25 0.25 0.25];
hold on;
threshold = yline(2*r_coll,'-.','LineWidth',1.5);
threshold.Color = [0.25 0.25 0.25];
xlabel('Time [s]','fontsize',fontsize);
ylabel('Distance [m]','fontsize',fontsize);
legend('Average','Reference','Threshold','fontsize',fontsize);

end
