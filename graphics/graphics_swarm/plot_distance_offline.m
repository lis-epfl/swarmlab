function [fig] = plot_distance_offline(pos_history, ...
    time_history, r_coll, d, r_comm, max_neig, fontsize, color)

% plot_distance_offline - Plot the min/avg/max distance of all the pairs of
% agents

x0 = 10; 
y0 = 10; 
width = 600;
height = 150;

[t_steps,~] = size(pos_history);
min_dist = zeros(t_steps,1);
max_dist = zeros(t_steps,1);
avg_dist = zeros(t_steps,1);

% Compute distances
for k = 1:t_steps
    pos_k = pos_history(k,:);
    pos_k = reshape(pos_k,3,[]);
    dist_k = pos2dist(pos_k);
    M = compute_neighborhood(dist_k, r_comm, max_neig);
    dist_neig = dist_k(M == true);
    if ~isempty(dist_neig)
        % Distance of all couples of neighboring agents (counted twice if
        % they see each other)
        min_dist(k) = min(dist_neig);
        max_dist(k) = max(dist_neig);
        avg_dist(k) = mean(dist_neig);
    else
        min_dist(k) = NaN;
        max_dist(k) = NaN;
        avg_dist(k) = NaN;
    end
end

% Create figure
fig = figure('Name','Offline swarm distances','NumberTitle','off');
hold on;
grid on;
box on;

% Plot envelope
err_bar(1,:,:) = [max_dist-avg_dist, avg_dist-min_dist];
if ~isempty(color)
    line_props.col = {color};
    mseb(time_history',avg_dist',err_bar,line_props);
else
    mseb(time_history',avg_dist',err_bar);
end

% Plot min-max
plot(time_history', min_dist, '--b','LineWidth', 1.0);
plot(time_history', max_dist, '--b','LineWidth', 1.0);

% Plot reference value
reference = yline(d,'--','LineWidth',1.5);
reference.Color = [0.25 0.25 0.25];

% Plot threshold
threshold = yline(2*r_coll,'-.','LineWidth',1.5);
threshold.Color = [0.25 0.25 0.25];

xlabel('Time [s]','fontsize',fontsize);
ylabel('Distance [m]','fontsize',fontsize);
h = legend('Average','Reference','Threshold','fontsize',fontsize);
set(h, 'location', 'northeastoutside');

set(fig,'units','pixels','position',[x0,y0,width,height]);
set(fig,'PaperPositionMode','auto');

end
