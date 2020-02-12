function plot_velocity_field( pos , vel , t)
% PLOT_VELOCITY_FIELD - Plot the velocity field of all agents of a swarm

persistent velocity_fig
persistent quiver_handle

[nb_dim, ~] = size(pos);

myblue      = [0 0 1]';

X       = pos(1, :)';
Y       = pos(2, :)';
U       = vel(1, :)';
V       = vel(2, :)';
if nb_dim == 3
    Z   = pos(3, :)';
    W   = vel(3, :)';
end

if t == 0
    
    velocity_fig = figure('Position',[660, 520, 560, 420]);
    clf(velocity_fig, 'reset');
    
    if nb_dim == 2
        quiver_handle = quiver(X, Y, U, V, 'color', myblue, 'LineWidth', 1.2);  
    elseif nb_dim == 3
        quiver_handle = quiver3(X, Y, Z, U, V, W, 'color', myblue, 'LineWidth', 1.2);
    end
    title(quiver_handle.Parent, 'Velocity field');

else % t > 0
    
    if nb_dim == 2
        quiver_handle = quiver(quiver_handle.Parent, X, Y, U, V, 'color', myblue, 'LineWidth', 1.2);
    elseif nb_dim == 3
        quiver_handle = quiver3(quiver_handle.Parent, X, Y, Z, U, V, W, 'color', myblue, 'LineWidth', 1.2);
    end
    title(quiver_handle.Parent, 'Velocity field');
    
end
    
end

