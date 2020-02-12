function [speed_handle] = plot_speed_offline(vel_history, time_history, ...
    v_swarm, v_norm_max, fontsize, color)

% PLOT_SPEED_OFFLINE - Plot the min/avg/max of the agents

speed_handle = figure('Name','Offline swarm speed','NumberTitle','off');
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

err_bar(1,:,:) = [max_speed-avg_speed, avg_speed-min_speed];
if ~isempty(color)
    line_props.col = {color};
    mseb(time_history',avg_speed',err_bar,line_props);
else
    mseb(time_history',avg_speed',err_bar);
end

hold on;
reference = yline(v_swarm,'--','LineWidth',1.5);
reference.Color = [0.25 0.25 0.25];
if ~isempty(v_norm_max)
    threshold = yline(v_norm_max,'-.','LineWidth',1.5);
    threshold.Color = [0.25 0.25 0.25];
end
xlabel('Time [s]','fontsize',fontsize);
ylabel('Speed [m/s]','fontsize',fontsize);
if ~isempty(v_norm_max)
    legend('Average','Reference','Threshold','fontsize',fontsize);
else
    legend('Average','Reference','fontsize',fontsize);
end

end
