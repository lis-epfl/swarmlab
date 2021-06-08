function total_wind = get_wind(time, map, time_step, sim_time, pos_ned, ...
                        wind_active, wind_gust_active, wind_level, ...
                        wind_gust_level)
                    
% get_wind - Get the wind depending on parameters and time.
% The wind varies between 0 and average_speed if wind_level is 100%
% the gust varies between 0 and max_gust_speed if wind_gust_level is 100%
%
% Inputs:
%   time
%   map
%   time_step
%   sim_time
%   pos_ned
%   wind_active
%   wind_gust_active
%   wind_level
%   wind_gust_level
%
% Outputs:
%
%

% TODO: For now the altitude doesn't have any effect as it would increase 
% computational time too much to recompute wind_struct each time...
% TODO: Gusts are not dependent on position: two drones side by side could
% have completely different gusts


persistent wind_struct

% Wind parameters
average_speed = 5;         % m/s
max_gust_speed = 2;        % m/s

% First call, creates wind structure for all the simulation
if wind_active && isempty(wind_struct)
    
    % Credits to PEF Mathworks user for the wind model generation functions
    % https://www.mathworks.com/matlabcentral/fileexchange/54491-3d-turbulent-wind-generation
    
    % define parameters
    U0 = average_speed;    % average wind speed 
    I0 = 10;               % turbulence intensity in %
    Seed = randi([1 2^31-1]);
    HubHt = 100;           % hub height
    Nx = 11;               % number of points on x axis (odd number)
    Ny = 11;               % number of points on y axis (odd number)
    Lx = map.width;        % grid x width in m
    Ly = map.width;        % grid y width in m
    dt = time_step;        % time step
    T = sim_time;          % simulation length
    xLu = 340.2;           % length scale in u
    xLv = 113.4;           % length scale in v
    xLw = 27.72;           % length scale in w
    Lc = 340.2;            % coherence length scale (default 340.2)
    a = 12;                % coherence decay (default 12)
    shearExp = 0.2;        % vertical wind shear exponent (default 0.2)
    
    % Create wind field structure
    [WF,~,t,~,~,X,Y,~,~] = turbulent_wind_field_generator(U0,I0,Seed,HubHt,Nx,Ny,Lx,Ly,dt,T,xLu,xLv,xLw,Lc,a,shearExp);
    wind_struct.WF = WF;
    wind_struct.time = t';
    wind_struct.X = flip(X(1,:)'+map.width/2);
    wind_struct.Y = Y(:,1);
end

if wind_active
    % Find current (closest) time step
    [~, time_index] = min(abs(time*ones(size(wind_struct.time))-wind_struct.time));

    % Find closest position
    [~, x_index] = min(abs(pos_ned(1)*ones(size(wind_struct.X))-wind_struct.X));
    [~, y_index] = min(abs(pos_ned(2)*ones(size(wind_struct.Y))-wind_struct.Y));

    % Get current 3x1 wind vector
    wind = wind_level * circshift((reshape(wind_struct.WF(time_index,x_index,y_index,:),3,1)), [0 -1]) / 100;

else
    wind = zeros(3,1);
end

if wind_gust_active
    gust = max_gust_speed * wind_gust_level * [normrnd(0,0.3); normrnd(0,0.3); normrnd(0,0.3)] / 100;  % random values between \pm max_gust_speed
    
else
    gust = zeros(3,1);
end

total_wind = [wind; gust];

end


