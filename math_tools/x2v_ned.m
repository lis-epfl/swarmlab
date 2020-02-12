function [v_ned] = x2v_ned(x)
    
    v_xyz = x(4:6);
    phi      = x(10);   % roll angle
    theta    = x(11);  % pitch angle
    psi    = x(12);      % yaw angle

    v_ned = rotate_b2i(v_xyz, phi, theta, psi) ;

end