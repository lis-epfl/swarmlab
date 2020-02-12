function [F, G, H, u, Q, R] = ekf_phi_theta(uu, P)

% EKF_PHI_THETA - Build the system matrices and inputs for the extended
% kalman filter extimating phi (roll) and theta (pitch)
% 
% Syntax:
%
% Inputs:
%
% Outputs:
%
%
% x = [phi; theta];
% u = [p; q; r; Va];
% xdot = F(x,u)*x + csi;
% y    = H(x,u)*x + eta;

% Estimated state for computing F,G,H
phi     = uu(1); 
theta   = uu(2);
p       = uu(3);
q       = uu(4);
r       = uu(5);
Va      = uu(6);

cp = cos(phi);
sp = sin(phi);
ct = cos(theta);
st = sin(theta);
tt = tan(theta);

F = [  q*cp*tt - r*sp*tt,   (q*sp-r*cp)/(ct^2); 
     - q*sp - r*cp,          0                ];
 
G = zeros(2, 4);
u = zeros(4,1);

H = [  0,                 q*Va*ct + P.gravity*ct;
     - P.gravity*cp*ct,   - r*Va*st - p*Va*ct + P.gravity*sp*st;
       P.gravity*sp*ct,   (q*Va+P.gravity*cp)*st               ];
    
Q = zeros(2);  % how to find the correlation??
R = zeros(3);  % ... ??
   
end

