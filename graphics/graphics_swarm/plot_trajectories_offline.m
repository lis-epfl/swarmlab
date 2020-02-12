function [traj_handle] = plot_trajectories_offline(pos_history, N, ...
                colors, fontsize, map)
% PLOT_TRAJECTORIES_OFFLINE - Plot trajectories of the agents

traj_handle = figure('Name','Offline swarm trajectories','NumberTitle','off');
if ~isempty(map)
    traj_handle = draw_cylinders(traj_handle,map);
end


for agent = 1:N
    hold on;
    
    if ~isempty(colors)
        plot3(pos_history(:,(agent-1)*3+2), pos_history(:,(agent-1)*3+1), ...
            - pos_history(:,(agent-1)*3+3), 'Color', colors(:,agent));
    else
        plot3(pos_history(:,(agent-1)*3+2), pos_history(:,(agent-1)*3+1), ...
            - pos_history(:,(agent-1)*3+3));
    end
    
end
% title('Swarm trajectories');
xlabel('Y Position [m]','fontsize',fontsize);
ylabel('X Position [m]','fontsize',fontsize);
zlabel('Z Position [m]','fontsize',fontsize);
view(2);


end
