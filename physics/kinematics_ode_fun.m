function dxdt = kinematics_ode_fun(t, x, xx, uu, P)
    
    % Speed and position
    vx    = x(4);
    vy    = x(5);
    vz    = x(6);
    p     = x(10);
    q     = x(11);
    r     = x(12);

    % Forces and moments
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    l     = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    % Orientation
    phi0   = xx(7);
    theta0 = xx(8);
    psi0   = xx(9);
    
    % Trigonometry
    cr = cos(phi0);
    cp = cos(theta0);
    sr = sin(phi0);
    tp = tan(theta0);
    
    % Rotation matrix from inertial frame to body frame
    Rbi = Rb2i(phi0, theta0, psi0);
    
    % Velocity in inertial frame
    pos_dot = Rbi* [vx vy vz]';
    pndot = pos_dot(1);  
    pedot = pos_dot(2);
    pddot = pos_dot(3);

    % Acceleration in body frame
    vxdot = r*vy - q*vz + fx/P.mass;
    vydot = p*vz - r*vx + fy/P.mass;
    vzdot = q*vx - p*vy + fz/P.mass;

    % Rotation matrix for rotation rate
    Si_b = [1, sr*tp, cr*tp;   ...
            0, cr,    -sr;     ...
            0, sr/cp,  cr/cp];

    % Rotation rate in bodyframe
    phidot   = Si_b(1,:) * [p q r]';
    thetadot = Si_b(2,:) * [p q r]';
    psidot   = Si_b(3,:) * [p q r]';

    % Rotation acceleration in body frame
    pdot = P.gamma1*p*q - P.gamma2*q*r + P.gamma3*l + P.gamma4*n;
    qdot = P.gamma5*p*r - P.gamma6*(p^2-r^2) + m/P.Jy;
    rdot = P.gamma7*p*q - P.gamma1*q*r + P.gamma4*l + P.gamma8*n;

    % Output
    dxdt = [pndot, pedot, pddot, vxdot, vydot, vzdot, ...
             phidot, thetadot, psidot, pdot, qdot, rdot]';

end