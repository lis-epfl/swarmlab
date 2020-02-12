function handle = draw_path(fig_handle, path, map_width, handle)
    flag = path(1); 
    r    = [path(3); path(4); path(5)];
    q    = [path(6); path(7); path(8)];
    c    = [path(9); path(10); path(11)];
    rho  = path(12);
    lambda  = path(13);

    switch flag
        case 1      % straight line
            XX = [r(1), r(1)+map_width*q(1)];
            YY = [r(2), r(2)+map_width*q(2)];
            ZZ = [r(3), r(3)+map_width*q(3)];
        case 2      % circular orbit
            N = 100;
            th = [0:2*pi/N:2*pi];
            XX = c(1) + rho*cos(th);
            YY = c(2) + rho*sin(th);
            ZZ = c(3)*ones(size(th));
    end
    
    if isempty(handle)
        handle = plot3(fig_handle.Children, YY,XX,-ZZ,'r');
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 