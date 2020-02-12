clc,clear,close all

U0 = 10;            % average hub height
I0 = 10;            % turbulence intensity in %
Seed = randi([1 2^31-1]);
HubHt = 114;        % hub height
Ny = 5;             % number of points on y axis (odd number)
Nz = 5;             % number of points on z axis (odd number)
Ly = 200;           % grid y width in m
Lz = 200;           % grid z width in m
dt = 0.05;          % time step
T = 100;            % simulation length
xLu = 340.2;        % length scale in u
xLv = 113.4;        % length scale in u
xLw = 27.72;        % length scale in u
Lc = 340.2;         % coherence length scale
a = 12;             % coherence decay
shearExp = 0.2;     % vertical wind shear exponent

%% TEST WIND FIELD GENERATION

tic
[WF,WFtower,t,dy,dz,Y,Z,Zbottom,Ztower] = ...
    turbulent_wind_field_generator(U0,I0,Seed,HubHt,Ny,Nz,Ly,Lz,dt,T,...
    xLu,xLv,xLw,Lc,a,shearExp);
toc

%% WRITE FAST-COMPATIBLE BTS FILE

% WindFileStruct.WF = WF;
% WindFileStruct.WFtower = WFtower;
% WindFileStruct.Ntower = numel(Ztower);
% WindFileStruct.Nz = Nz;
% WindFileStruct.Ny = Ny;
% WindFileStruct.N = numel(t);
% WindFileStruct.dz = dz;
% WindFileStruct.dy = dy;
% WindFileStruct.dt = (t(2) - t(1));
% WindFileStruct.U0 = U0;
% WindFileStruct.HubHt = HubHt;
% WindFileStruct.Zbottom = Zbottom;
% WindFileStruct.fileID = 'TestTurbulentWindGeneration';
% WindFileStruct.Version = ['Dummy Turbulent Wind Generator' datestr(clock,0)];
% 
% writeBTSfile(WindFileStruct);