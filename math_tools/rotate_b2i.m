function [ u_i ] = rotate_b2i( u_b, phi, theta, psi )
% ROTATE_B2I - Rotation of an object (vector/matrix) from the body to the 
% inertial 3D-space
%
%
%

    cr = cos(phi);
    cp = cos(theta);
    cy = cos(psi);
    sr = sin(phi);
    sp = sin(theta);
    sy = sin(psi);
    
    % Rotation matrix from inertial frame to body frame

    Rbi = [cp*cy, sr*sp*cy-cr*sy, cr*sp*cy+sr*sy;...
            cp*sy, sr*sp*sy+cr*cy, cr*sp*sy-sr*cy;...
            -sp,   sr*cp,          cr*cp];
        
    u_i = Rbi*u_b;
        

end

