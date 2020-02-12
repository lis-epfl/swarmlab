function [ y ] = lp_filter ( y_prev, u, a, dt)
% LP_FILTER implements a low pass filter.
%
% Syntax: [ output ] = lp_filter ( input, a, dt)
%
% Inputs:
%   y_prev - previous output value
%   u      - signal value
%   a      - time constant
%   Ts     - time step interval

y = exp(-a*dt)*y_prev + (1-exp(-a*dt))*u;


end

