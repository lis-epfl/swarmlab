function [fig_handle] = plot_swarm_performance(time_history, safety, ...
    order, union, alg_conn, safety_obs, min_d_obs, p_swarm, fontsize, dirname)
% Plot swarm performance - plot the performance functions defined for the
% swarm, namely: the safety, order, union and connectivity.

fig_handle = figure('Name','Swarm performance analyser','NumberTitle','off');
plot(time_history, safety, 'LineWidth', 1.5);
hold on;
plot(time_history, order, 'LineWidth', 1.5);
hold on;
plot(time_history, union, 'LineWidth', 1.5);
hold on;
plot(time_history, alg_conn, 'LineWidth', 1.5);
hold on;
plot(time_history, safety_obs, 'LineWidth', 1.5);
hold on;
xlabel('Time [s]', 'fontsize', fontsize);
ylabel('Performance', 'fontsize', fontsize);
legend('safety','order','union','connectivity', 'safety obsacles');

figure;
plot(time_history, min(min_d_obs,[],2), 'LineWidth', 1.5);
yline(0,'LineWidth', 1.5);
xlabel('Time [s]', 'fontsize', fontsize);
ylabel('Distance to obstacles', 'fontsize', fontsize);

% Save only if 'dirname' is different from '[]'
if ~isempty(dirname)
    file_path = strcat(dirname,'/performance');
    savefig(fig_handle,file_path);
    print(fig_handle,file_path,'-dpng','-r300');
end

end