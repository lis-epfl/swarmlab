function draw_buildings(map)
% DRAW_BUILDINGS - plot parallelepipadal buildings
  
  vertices = [];
  faces = [];
  patch_colors = [];
  count = 0;
  for i=1:length(map.buildings_north)
      [vertices_temp,faces_temp,patch_colors_temp] = building_vert_face(...
          map.buildings_north(i), map.buildings_east(i),...
          map.building_width, map.buildings_heights(i));
      vertices = [vertices; vertices_temp];
      faces_temp = faces_temp + count;
      faces = [faces; faces_temp];
      count = count + 8;
      patch_colors = [patch_colors;patch_colors_temp];
  end
  
  patch('Vertices', vertices, 'Faces', faces,...
      'FaceVertexCData',patch_colors,...
      'FaceColor','flat');
  axis_len = map.width;
  margin = axis_len/5;
  axes_lim = [-margin, axis_len + margin, ...
      -margin, axis_len+margin, ...
      0, 1.2 * map.max_height];
  axis(axes_lim);
  axis square;
  view(32, 47);
  hold on;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vertices,faces,patch_colors] = building_vert_face(n,e,width,height)
% BUILDING_VERT_FACE - define patches for a building located at (n,e)
 
  % Vertices of the building
  vertices = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % Define faces of fuselage
  faces = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  my_gray = [0.7, 0.7, 0.7];
  my_light_gray = [0.9, 0.9, 0.9];

  patch_colors = [...
    my_gray;...      % North
    my_gray;...      % East
    my_gray;...      % South
    my_gray;...      % West
    my_light_gray;... % Top
    ];

end
