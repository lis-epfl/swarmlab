function zhat = true_states(x)
%
% TRUE_STATES - Fake state estimation for mavsim. This function will be
% replaced with a state estimator in a later chapter.
%
% Syntax: xhat = estimate_states(x,P)
%
% Inputs:
%   x        - state variables
%   P        - parameters
%
% Outputs:
%   pnhat    - estimated North position     [m]
%   pehat    - estimated East position      [m]
%   hhat     - estimated altitude           [m]
%   Vahat    - estimated airspeed           [m/s]
%   alphahat - estimated angle of attack    [rad]
%   betahat  - estimated sideslip angle     [rad]
%   phihat   - estimated roll angle         [rad]
%   thetahat - estimated pitch angle        [rad]
%   chihat   - estimated course             [rad]
%   phat     - estimated roll rate          [rad/s]
%   qhat     - estimated pitch rate         [rad/s]
%   rhat     - estimated yaw rate           [rad/s]
%   Vghat    - estimated ground speed       [m/s]
%   wnhat    - estimate of North wind       [m/s]
%   wehat    - estimate of East wind        [m/s]
%   bxhat    - estimate of x-gyro bias      []
%   byhat    - estimate of y-gyro bias      []
%   bzhat    - estimate of z-gyro bias      []
%

    % State variables
    NN = 0;
    pn       = x(1+NN);  % inertial North position
    pe       = x(2+NN);  % inertial East position
    h        = -x(3+NN); % altitude
    vx       = x(4+NN);  % inertial velocity along body x-axis
    vy       = x(5+NN);  % inertial velocity along body y-axis
    vz       = x(6+NN);  % inertial velocity along body z-axis
    phi      = x(7+NN);  % roll angle
    theta    = x(8+NN);  % pitch angle
    psi      = x(9+NN);  % yaw angle
    p        = x(10+NN); % body frame roll rate
    q        = x(11+NN); % body frame pitch rate
    r        = x(12+NN); % body frame yaw rate
    NN = NN+12;
    
    % Air data
    Va       = x(1+NN);  % airspeed
    alpha    = x(2+NN);  % angle of attack
    beta     = x(3+NN);  % sideslip angle
    wn       = x(4+NN);  % wind North
    we       = x(5+NN);  % wind East
    % wd     = x(6+NN);  % wind down
    NN = NN+6;
    
    % Time
    % t      = x(1+NN);   % time
    
    % Estimated states (using real state data)
%     pnhat      = pn;
%     pehat      = pe;
%     hhat       = h;
%     uhat       = vx;
%     vhat       = vy;
%     what       = vz;
%     Vahat      = Va;
%     alphahat   = alpha;
%     betahat    = beta;
%     phihat     = phi;
%     thetahat   = theta;
%     psihat     = psi;
    chihat     = atan2(Va*sin(psi)+we, Va*cos(psi)+wn);
%     phat       = p;
%     qhat       = q;
%     rhat       = r;
    Vghat      = sqrt((Va*cos(psi)+wn)^2 + (Va*sin(psi)+we)^2);
    wnhat      = wn;
    wehat      = we;
%     psihat     = psi;
%     bxhat    = P.bias_gyro_x;
%     byhat    = P.bias_gyro_y;
%     bzhat    = P.bias_gyro_z;
    bxhat      = 0;
    byhat      = 0;
    bzhat      = 0;
    
    
    zhat = [...
        pn;...
        pe;...
        h;...
        vx;...
        vy;...
        vz;...
        Va;...
        alpha;...
        beta;...
        phi;...
        theta;...
        psi; ...
        chihat;...
        p;...
        q;...
        r;...
        Vghat;...
        wnhat;...
        wehat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
    
end 