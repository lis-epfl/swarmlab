function [fig] = plot_acceleration_offline(accel_history, ...
    time_history, accel_norm_max, fontsize, color)

% plot_acceleration_offline - Plot the min/avg/max acceleration of the 
% agents

x0 = 10; 
y0 = 10; 
width = 600;
height = 150;

t_steps = length(accel_history(:,1));
min_accel = zeros(t_steps,1);
max_accel = zeros(t_steps,1);
avg_accel = zeros(t_steps,1);

for k = 1:t_steps
    Accel_k = accel_history(k,:);
    Accel_k = reshape(Accel_k,3,[]);
    Accel_norm_k = sqrt(sum(Accel_k.^2,1));
    min_accel(k) = min(Accel_norm_k);
    max_accel(k) = max(Accel_norm_k);
    avg_accel(k) = mean(Accel_norm_k);
end

fig = figure('Name','Offline swarm acceleration','NumberTitle','off');
hold on;
grid on;
box on;

% Plot envelope
err_bar(1,:,:) = [max_accel-avg_accel, avg_accel-min_accel];
if ~isempty(color)
    line_props.col = {color};
    mseb(time_history(1:t_steps)',avg_accel',err_bar,line_props);
else
    mseb(time_history(1:t_steps)',avg_accel',err_bar);
end

% Plot reference value
reference = yline(0,'--','LineWidth',1.5);
reference.Color = [0.25 0.25 0.25];
if ~isempty(accel_norm_max)
    threshold = yline(accel_norm_max,'-.','LineWidth',1.5);
    threshold.Color = [0.25 0.25 0.25];
end

xlabel('Time [s]','fontsize',fontsize);
ylabel('Acceleration [m/s^2]','fontsize',fontsize);
if ~isempty(accel_norm_max)
    h = legend('Average', 'Reference', 'Threshold','fontsize',fontsize);
else
    h = legend('Average', 'Reference','fontsize',fontsize);
end
set(h, 'location', 'northeastoutside');

set(fig,'units','pixels','position',[x0,y0,width,height]);
set(fig,'PaperPositionMode','auto');

end