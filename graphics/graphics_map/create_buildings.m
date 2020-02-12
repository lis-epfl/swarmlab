function map = create_buildings(map)
% CREATE_BUILDINGS - Create the city buildings in grid. The heights are
% random.
%
% Inputs:
%   map: structure of map parameters
%  
% Outputs:
%   map: structure of map parameters
%
    
    map.buildings_heights = map.max_height*rand(map.nb_blocks*map.nb_blocks,1);
    
    for i=1:map.nb_blocks
        buildings_north(i) = [0.5*map.width/map.nb_blocks*(2*(i-1)+1)];
    end
    map.buildings_north = repmat(buildings_north', map.nb_blocks, 1);
    map.buildings_east  = repmat(buildings_north, map.nb_blocks, 1);
    map.buildings_east  = map.buildings_east(:);

end