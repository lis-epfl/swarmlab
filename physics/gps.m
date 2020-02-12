function y = gps(x, P)
% GPS - Compute the output of gps sensor.
%
% Syntax: y = gps(x, P)
%
% Inputs:
%   x - state variables
%   P - parameters
%
% Outputs:
%   y - 
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

    % Relabel the inputs
        Va      = x(1);
    %   alpha   = x(2);
    %   beta    = x(3);
        wn      = x(4);
        we      = x(5);
    %   wd      = x(6);
        pn      = x(7);
        pe      = x(8);
        pd      = x(9);
        vx      = x(10);
        vy      = x(11);
        vz      = x(12);
        phi     = x(13);
        theta   = x(14);
        psi     = x(15);
    %   p       = x(16);
    %   q       = x(17);
    %   r       = x(18);
        t       = x(19);
    
    % Construct Markov Process
    
    persistent mp_n;
    persistent mp_e;
    persistent mp_h;
    if isempty(mp_n)
        mp_n=0;
        mp_e=0;
        mp_h=0;
    end
    mp_n = exp(-P.k_gps*P.dt)*mp_n + normrnd(0, P.sd_gps_pn);
    mp_e = exp(-P.k_gps*P.dt)*mp_e + normrnd(0, P.sd_gps_pe);
    mp_h = exp(-P.k_gps*P.dt)*mp_h + normrnd(0, P.sd_gps_ph);
    
    
    % Construct North, East, and altitude GPS measurements
    y_gps_pn =   pn + mp_n;
    y_gps_pe =   pe + mp_e; 
    y_gps_ph = - pd + mp_h; 
    
    Rbi = Rb2i(phi, theta, psi);
    vi = Rbi * [vx; vy; vz];
    vn = vi(1);
    ve = vi(2);
    vd = vi(3);

    y_gps_vn =   vn + normrnd(0, P.sd_gps_vn);
    y_gps_ve =   ve + normrnd(0, P.sd_gps_ve); 
    y_gps_vh = - vd + normrnd(0, P.sd_gps_vh);
    
    % construct groundspeed and course measurements
    y_gps_Vg     = sqrt(y_gps_vn^2 + y_gps_ve^2);
    y_gps_course = atan2(y_gps_ve, y_gps_vn);

    % construct total output
    y = [y_gps_pn; y_gps_pe; y_gps_ph; y_gps_Vg; y_gps_course];
    
end



