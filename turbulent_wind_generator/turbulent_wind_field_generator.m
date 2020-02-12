% Copyright (c) 2015, Francesco Perrone
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are
% met:
% 
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in
%       the documentation and/or other materials provided with the distribution
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.

function [WF,WFtower,t,dy,dz,Y,Z,Zbottom,Ztower] = turbulent_wind_field_generator(U0,I0,Seed,HubHt,Ny,Nz,Ly,Lz,dt,T,xLu,xLv,xLw,Lc,a,shearExp)

%% FREQUENY/TIME PREALLOCATION & INITIALISATION
T = round(T + (Ly/U0));
Nt = Ny*Nz; % Total number of grid points
N = find_steps_number(T,dt); % Number of samples (in frequency and time domain)
Fs = N/T;
df = Fs/N; % Frequency sample
fp = df:df:0.5*Fs; % Positive frequency array [Hz]
dT = T/N; % Wind file time step

%% DEFINITION OF SIMULATION GRID

[Y,Z,dy,dz] = define_grid_coordinates(Ly,Lz,Ny,Nz,HubHt); % Generate (Y,Z) coordinates of grid points and spacing on y-/z-axis
Zunique = unique(Z);
Zbottom = min(Zunique(:));
Ztower = (Zbottom):-dz:0;
% Ytower = zeros(size(Ztower));
Ntower = numel(Ztower);
Uvert = repmat(U0.*((Zunique/HubHt).^shearExp),1,Ny)'; % Exponential vertical wind profile
Uvert_tower = U0.*((Ztower/HubHt).^shearExp);
Uvert1 = Uvert(:);
Distance = abs(sqrt(bsxfun(@plus,sum([Y(:) Z(:)].^2,2),sum([Y(:) Z(:)].^2,2)') - 2*([Y(:) Z(:)]*[Y(:) Z(:)]'))); % Distance matrix

%% POWER SPECTRA

r = 5/3; % Kaimal spectrum exponent
sigma_u = (I0*0.01*U0);
sigma_v = 0.8*sigma_u;
sigma_w = 0.5*sigma_u;
var_u = sigma_u^2;
var_v = sigma_v^2;
var_w = sigma_w^2;

Suu_p = ((4*var_u*(xLu/U0))./((1 + 6*fp*(xLu/U0)).^r))*0.5; % Positive amplitude spectrum (U-component)
Svv_p = ((4*var_v*(xLv/U0))./((1 + 6*fp*(xLv/U0)).^r))*0.5; % Positive amplitude spectrum (V-component)
Sww_p = ((4*var_w*(xLw/U0))./((1 + 6*fp*(xLw/U0)).^r))*0.5; % Positive amplitude spectrum (W-component)

Su = sqrt(Suu_p*N*Fs);
Sv = sqrt(Svv_p*N*Fs);
Sw = sqrt(Sww_p*N*Fs);

%% CALCULATION OF FFT TERMS

rng(Seed,'twister'); % Start Mersenne twister

fft_uu_p = zeros(Nt,N*0.5);
fft_vv_p = zeros(Nt,N*0.5);
fft_ww_p = zeros(Nt,N*0.5);

nn_u = exp(1i*2*pi*rand(Nt,numel(fp)));
nn_v = exp(1i*2*pi*rand(Nt,numel(fp)));
nn_w = exp(1i*2*pi*rand(Nt,numel(fp)));

for idx_chol = 1:numel(fp)
    Coh_uu = chol(exp(-a.*Distance.*sqrt(((fp(idx_chol)./U0).^2) + ((0.12./Lc).^2))),'lower'); %Coherence function U-component (according to IEC 61400-1 Ed.3)
    fft_uu_p(:,idx_chol) = (Coh_uu*Su(idx_chol))*nn_u(:,idx_chol);%chol(Coh_uu,'lower')
    fft_vv_p(:,idx_chol) = (Coh_uu*Sv(idx_chol))*nn_v(:,idx_chol);%chol(Coh_vv_ww,'lower')
    fft_ww_p(:,idx_chol) = (Coh_uu*Sw(idx_chol))*nn_w(:,idx_chol);%chol(Coh_vv_ww,'lower')
end

% Prepare FFT terms for each time series by mirroring the positive side
% about the frequency axis
fft_uu = [zeros(Nt,1) fft_uu_p fliplr(conj(fft_uu_p(:,1:end-1)))];
fft_vv = [zeros(Nt,1) fft_vv_p fliplr(conj(fft_vv_p(:,1:end-1)))];
fft_ww = [zeros(Nt,1) fft_ww_p fliplr(conj(fft_ww_p(:,1:end-1)))];

fft_uu(:,N*0.5 + 1) = real(fft_uu(:,N*0.5 + 1));
fft_vv(:,N*0.5 + 1) = real(fft_vv(:,N*0.5 + 1));
fft_ww(:,N*0.5 + 1) = real(fft_ww(:,N*0.5 + 1));

%% GENERATE TIME SERIES

Ucomp = (ifft((fft_uu),[],2));
Vcomp = (ifft((fft_vv),[],2));
Wcomp = (ifft((fft_ww),[],2));

%% SCALE TIME SERIES

stdU = (std(Ucomp,[],2));
stdV = (std(Vcomp,[],2));
stdW = (std(Wcomp,[],2));

idx_hub = ((Nz+1)*Ny)*0.5 - (Ny-1)*0.5;

SF = [sigma_u./stdU(idx_hub) , sigma_v./stdV(idx_hub) , sigma_w./stdW(idx_hub)];

Ucomp1 = arrayfun(@(i) Ucomp(i,:)*SF(1),1:Nt,'UniformOutput',false);
Ucomp1 = vertcat(Ucomp1{:});
Vcomp1 = arrayfun(@(i) Vcomp(i,:)*SF(2),1:Nt,'UniformOutput',false);
Vcomp1 = vertcat(Vcomp1{:});
Wcomp1 = arrayfun(@(i) Wcomp(i,:)*SF(3),1:Nt,'UniformOutput',false);
Wcomp1 = vertcat(Wcomp1{:});

t = 0:dT:(N-1)*dT;

WF = zeros(N,Nz,Ny,3);
WFtower = zeros(N,numel(Ztower),3);
it = 0;
for iz = 1:Nz
    for iy = 1:Ny
        it = it + 1;
        WF(:,iz,iy,1) = Ucomp1(it,:) + Uvert1(it);
        WF(:,iz,iy,2) = Vcomp1(it,:);
        WF(:,iz,iy,3) = Wcomp1(it,:);
    end
end


for iz = 1:Ntower   
        WFtower(:,iz,1) = (WF(:,1,.5*(Ny+1),1) - mean(WF(:,1,.5*(Ny+1),1))) + Uvert_tower(iz);
        WFtower(:,iz,2) = WF(:,1,.5*(Ny+1),2);
        WFtower(:,iz,3) = WF(:,1,.5*(Ny+1),3);    
end

end