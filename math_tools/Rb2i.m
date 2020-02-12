function [ Rbi ] = Rb2i( phi, theta, psi )
%RIB Summary of this function goes here
%   Detailed explanation goes here
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
        

end

