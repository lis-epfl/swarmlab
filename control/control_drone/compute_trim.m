function [xyu_trimmed, options] = compute_trim(filename, Va, gamma, R, P)
% compute_trim - 
%
% Inputs:
%   Va: desired airspeed (m/s)
%   gamma: desired flight path angle (radians)
%   R: desired radius (m) - use (+) for right handed orbit, 
%                                 (-) for left handed orbit
%   P: set of parameters
%

% TODO: check if we can move this function to 'old' folder

% Inputs for trim function
if R~=Inf 
    dx = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va*cos(gamma)/R; 0; 0; 0];
else
    dx = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; 0; 0; 0; 0];
end

idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];
if R~=0 
    x0 = [0; 0; 0; Va; 0; 0; atan2(Va^2*cos(gamma),R*P.gravity); gamma; 0; 0; 0; 0];
else
    x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
end
ix0 = [];
% Help the trim function to converge
u0 = [0; 0; 0; 0.5];
iu0 = [];
y0 = [Va; gamma; 0];
iy0 = [1;3];

% Compute trim conditions
% options = [0; 10^(-8); 10^(-8); 10^(-8); 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 10^(-8); 0.1; 0];
[x_trim,u_trim,y_trim,dx_trim, options] = trim(filename,x0,u0,y0,ix0,iu0,iy0,dx,idx);

% Package trimmed state in only one vector
xyu_trimmed = zeros(19,1);
xyu_trimmed(1:12) = x_trim;
xyu_trimmed(13:15) = y_trim;
xyu_trimmed(16:19) = u_trim;

% Check to make sure that the linearization worked (should be small)
if norm(dx_trim(3:end)-dx(3:end))> 10^(-1)
    disp 'Error exceed the fixed limit'
end

end