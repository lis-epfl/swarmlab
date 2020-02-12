function out = path_manager_wing_chap10(in)

% PATH_MANAGER_WING_CHAP10
%
% Outputs:
%   flag_line - if flag==1, follow waypoint path
%               if flag==2, follow orbit
%   
%   Va_d    - desired airspeed
%   r       - inertial position of start of waypoint path
%   q       - unit vector that defines inertial direction of waypoint path
%   c       - center of orbit
%   rho     - radius of orbit
%   lambda  - direction of orbit (+1 for CW, -1 for CCW)
%

  NN = 0;
  flag_line = in(1+NN);
  NN = 1;
  wpt_list = in(1+NN);
  NN = NN + 1;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  Va        = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  % chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 

   if flag_line == 1     % case straight line
    % Define waypoint path
    Va_d   = 35;
    r      = [0; 0; -100];
    q      = [1/2; 1; 0];
    q      = q/norm(q);
    c      = [0;0;0];           % not used for waypoint path
    rho    = 0;                 % not used for waypoint path
    lambda = 0;                 % not used for waypoint path

   else             % flag=2, case circular orbit
    % Define orbit
    flag_line   = 2;
    Va_d   = 35;             % desired airspeed
    r      = [0; 0; 0];         % not used for orbit
    q      = [1; 0; 0];         % not used for orbit
    q      = q/norm(q);
    c      = [1;1;-100];        % center of orbit
    rho    = 400;               % radius of orbit
    lambda = 1;                 % direction of orbit

   end
   
    out = [flag_line; Va_d; r; q; c; rho; lambda];

end