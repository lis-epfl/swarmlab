function draw_spheres(time, period, S, figure_handle)
% DRAW_SPHERES - Plot the spherical obstacles

persistent scatter_handle

if mod(time, period) == 0 && S.is_active_spheres
    [~,n_obs] = size(S.S);
    s = repmat(400*600,n_obs,1);
    X = S.S(1,:)';
    Y = S.S(2,:)';
    Z = S.S(3,:)';
    colors = repmat(0.5,3, n_obs);

    if time == 0 
        scatter_handle = scatter3(X, Y, Z, s, colors', '.');
        hold on
        
        axis square;
        view(32,47);
        xlabel('x position [m]');
        ylabel('y position [m]');
        zlabel('z position [m]');
        
%     else
%         set(scatter_handle, 'Xdata', Y, ...
%             'Ydata', X, ...
%             'Zdata', Z, ...
%             'Marker', '.', ...
%             'SizeData', s, ...
%             'CData', colors');

    end
end
    
    