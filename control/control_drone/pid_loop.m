function [out, param] = pid_loop(y_c, y, dy, kp, ki, kd, limit, Ts, tau, param)

integrator      = param.int;
differentiator  = param.diff;
error_prev      = param.error_prev;
y_c_prev        = param.y_c_prev;


% Update error
error           = y_c - y; 

% Update integrator
if isfinite(dy)
    integrator  = integrator + Ts*(error + dy/2);
else
    integrator  = integrator + (Ts/2)*(error + error_prev);
end

% Update differentiator
if isfinite(1/kd)
    if isfinite(dy)
        dy_c = y_c - y_c_prev;          % Compute command diff
        derror = dy_c - dy;             % Compute error diff
        y_c_prev = y_c;                 % Update the command for next step
    else
        derror = error - error_prev;    % Compute error diff
        error_prev = error;             % Update the error for next step
    end
    differentiator = (2*tau-Ts)/(2*tau+Ts)*differentiator...
                        + 2/(2*tau+Ts)*(derror);
end

% Update output before considering saturation
out_unsat = kp*error + ki*integrator + kd*differentiator;
% Check saturation
out = saturate(out_unsat, limit);

% Implement integrator anti-windup
    if ki~=0
        integrator = integrator + Ts/ki * (out - out_unsat);
    end
    
param.int           = integrator;
param.diff          = differentiator;
param.error_prev    = error_prev;
param.y_c_prev      = y_c_prev;
    
    
end

function out = saturate(out_unsat, limit)

    if out_unsat > limit 
        out = limit;
    elseif out_unsat < -limit 
        out = -limit;
    else
        out = out_unsat;
    end
    
end