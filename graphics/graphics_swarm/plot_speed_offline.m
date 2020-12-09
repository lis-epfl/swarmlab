function [fig] = plot_speed_offline(vel_history, time_history, ...
    v_swarm, v_norm_max, fontsize, color)

% plot_speed_offline - Plot the min/avg/max of the agents.

x0 = 10; 
y0 = 10; 
width = 600;
height = 150;

t_steps = length(vel_history(:,1));
min_speed = zeros(t_steps,1);
max_speed = zeros(t_steps,1);
avg_speed = zeros(t_steps,1);

for k = 1:t_steps
    Vel_k = vel_history(k,:);
    Vel_k = reshape(Vel_k,3,[]);
    Speed_k = sqrt(sum(Vel_k.^2,1));
    min_speed(k) = min(Speed_k);
    max_speed(k) = max(Speed_k);
    avg_speed(k) = mean(Speed_k);
end

% Create figure
fig = figure('Name','Offline swarm speed','NumberTitle','off');
hold on;
grid on;
box on;

% Plot envelope
err_bar(1,:,:) = [max_speed-avg_speed, avg_speed-min_speed];
if ~isempty(color)
    line_props.col = {color};
    mseb(time_history',avg_speed',err_bar,line_props);
else
    mseb(time_history',avg_speed',err_bar);
end

% Plot reference value
reference = yline(v_swarm,'--','LineWidth',1.5);
reference.Color = [0.25 0.25 0.25];

% Plot threshold value
if ~isempty(v_norm_max)
    threshold = yline(v_norm_max,'-.','LineWidth',1.5);
    threshold.Color = [0.25 0.25 0.25];
end

xlabel('Time [s]','fontsize',fontsize);
ylabel('Speed [m/s]','fontsize',fontsize);
if ~isempty(v_norm_max)
    h = legend('Average','Reference','Threshold','fontsize',fontsize);
else
    h = legend('Average','Reference','fontsize',fontsize);
end
set(h, 'location', 'northeastoutside');

set(fig,'units','pixels','position',[x0,y0,width,height]);
set(fig,'PaperPositionMode','auto');

end
