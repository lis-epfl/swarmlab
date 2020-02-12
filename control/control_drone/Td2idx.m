function [Va_idx, gamma_idx, R_idx] = Td2idx(Td)
% Td2idx

Va_trim = Td(1);
gamma_trim = Td(2);
R_trim = Td(3);

if ~exist('P_Td', 'var')
    param_table_T
end
Va_idx = floor((Va_trim - P_Td.Va_min) / P_Td.Va_step)+1;
gamma_idx = floor((gamma_trim - P_Td.gamma_min) / P_Td.gamma_step)+1;
if ~isfinite(R_trim)
    R_idx = 1;
else
    R_idx = floor((R_trim - P_Td.R_min) / P_Td.R_step)+2;
end

if Va_idx > P_Td.Va_n_steps
    Va_idx = P_Td.Va_n_steps;
    str = ['Va_trim ' num2str(Va_trim) 'm/s too big.'];
    disp(str)
elseif Va_idx < 1
    Va_idx = 1;
    str = ['Va_trim ' num2str(Va_trim) 'm/s too small.'];
    disp(str)
end

if gamma_idx > P_Td.gamma_n_steps
    gamma_idx = P_Td.gamma_n_steps;
    str = ['gamma_trim ' num2str(rad2deg(gamma_trim)) ' too big.'];
    disp(str)
elseif gamma_idx < 1
    gamma_idx = 1;
    str = ['gamma_trim ' num2str(rad2deg(gamma_trim)) ' too small.'];
    disp(str)
end

if R_idx > P_Td.R_n_steps
    R_idx = P_Td.R_n_steps;
    str = ['R_trim ' num2str(R_trim) 'm too big.'];
    disp(str)
elseif R_idx < 1
    R_idx = 1;
    str = ['R_trim ' num2str(R_trim) 'm too small.'];
    disp(str)
end


end

