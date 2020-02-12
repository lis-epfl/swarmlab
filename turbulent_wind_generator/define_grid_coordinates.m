function [Y,Z,dy,dz] = define_grid_coordinates(Ly,Lz,Ny,Nz,HubHt)
dy = Ly/(Ny-1); % Spacing along Y axis
dz = Lz/(Nz-1); % Spacing along Z axis
if isequal(mod(Ny,2),0)
    iky = flip([(-Ny/2:-1) (1:Ny/2)]);
else
    iky = flip(-floor(Ny/2):ceil(Ny/2-1));
end

if isequal(mod(Nz,2),0)
    ikz = [(-Nz/2:-1) (1:Nz/2)];
else
    ikz = -floor(Nz/2):ceil(Nz/2-1);
end

% define Y and Z coordinates for each grid-point
[Y,Z] = ndgrid(iky*dy,(ikz*dz + HubHt));
Y = Y';
Z = Z';
end