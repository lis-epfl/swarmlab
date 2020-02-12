function [kf] = ext_kalman_filter(F, G, H, u, Q, R, time, last_measure, kf, P)

%EXT_KALMAN_FILTER - This is the extended Kalman filter.
%
% Synthax: 
%
% Inputs:
%   uu          - state variables, delta, wind
%   kf.x_prev   - previous a posteriori state estimate
%   kf.P_prev   - previous a posteriori estimate covariance
%   last_measure- (time, sensor measure)
%   P           - parameters
%   
% Outputs:
%   kf.x        - a priori/a posteriori state estimate
%   kf.P        - a priori/a posteriori estimate covariance
%   kf.z_res    - post-fit residual

% A -> F
% B -> G
% C -> H

% Q - covariance of the zero-mean Gaussian noise on x_dot
% R - covariance of the zero-mean Gaussian noise on y

time_m = last_measure(:,1);
z_m    = last_measure(:,2);

% prediction (a priori)
kf.x = F * kf.x_prev    + G * u;    % a priori state estimate
kf.P = F * kf.P_prev * (F') + Q;    % a priori estimate covariance

% update (a posteriori)
if time - time_m < P.dt
    kf.z_tilde = z_m - H*kf.x;      % innovation (a priori residual)
    S = R + H*kf.P*(H');            % innovation covariance
    K = kf.P*(H')/S;                 % optimal kalman gain
    kf.x = kf.x + K*kf.z_tilde;     % a posteriori state estimate
    kf.P = kf.P - K*S*(K');         % a posteriori estimate covariance
end

kf.z_res = z_m - H*kf.x;            % a posteriori residual

end

