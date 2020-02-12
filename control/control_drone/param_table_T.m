P_Td.Va_min          = 35; %[m/s]
P_Td.Va_max          = 35;
P_Td.Va_step         = 2;
P_Td.Va_n_steps      = max(floor((P_Td.Va_max-P_Td.Va_min) / P_Td.Va_step+1),1);

P_Td.gamma_min       = deg2rad(0); %[deg]
P_Td.gamma_max       = deg2rad(0);
P_Td.gamma_step      = deg2rad(1);
P_Td.gamma_n_steps   = max(floor((P_Td.gamma_max-P_Td.gamma_min) / P_Td.gamma_step+1),1);

P_Td.R_min           = 50; %[m]
P_Td.R_max           = 50;
P_Td.R_step          = 2;
P_Td.R_n_steps       = max(floor((P_Td.R_max-P_Td.R_min) / P_Td.R_step+1),1) + 1;


% T_trim = 25 + 273.15; % [Kelvin]
% p_trim = 1013.25;     % [hPa]
% hygrometry_trim = 50;   % [% air humidity]

