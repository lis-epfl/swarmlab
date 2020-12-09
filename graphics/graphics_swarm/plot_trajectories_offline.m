function [fig] = plot_trajectories_offline(pos_history, N, ...
                colors, fontsize, map)
% plot_trajectpries_offline - Plot trajectories of the agents.

x0 = 10; 
y0 = 10; 
width = 400;
height = 400;

fig = figure('Name','Offline swarm trajectories','NumberTitle','off');
hold on;
grid on;
box on;

% Plot environment
if ~isempty(map)
    fig = draw_cylinders(fig,map);
end

% Plot trajetcories
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

xlabel('Y Position [m]','fontsize',fontsize);
ylabel('X Position [m]','fontsize',fontsize);
zlabel('Z Position [m]','fontsize',fontsize);
view(2);

set(fig,'units','pixels','position',[x0,y0,width,height]);
set(fig,'PaperPositionMode','auto');

end
