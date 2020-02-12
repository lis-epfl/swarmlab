function [vertices,faces,face_colors] = define_quad_body()

% DEFINE_QUAD_BODY - Define vertices, faces and face colors for a 
% quadcopter
%
% Syntax: [vertices,faces,face_colors] = define_quad_body()
%
% Outputs:
%   vertices    - vertices
%   faces       - faces
%   face_colors - colors for drone faces


% Define colors
myred    = [1, 0, 0];
myblack  = [0, 0, 0];
mygray  = [0.7, 0.7, 0.7];

% Define the physical location of vertices
rot_angle = pi/4;
Rot = [cos(rot_angle), sin(rot_angle), 0;
      -sin(rot_angle), cos(rot_angle), 0;
      0              , 0             , 1];

vertices = Rot * [...
    1, 0, 0;...     % pt 1
    4, 3, 0;...     % pt 2
    3, 4, 0;...     % pt 3
    0, 1, 0;...     % pt 4
    -3, 4, 0;...    % pt 5
    -4, 3, 0;...    % pt 6
    -1, 0, 0;...    % pt 7
    -4, -3, 0;...   % pt 8
    -3, -4, 0;...   % pt 9
    0, -1, 0;...     % pt 10
    3, -4, 0;...     % pt 11
    4, -3, 0;...     % pt 12
    1, 0, 0;...      % pt 13
    0, 0, -1;...    % pt 14
    0, 1, 0;...      % pt 15
    0, 0, -1;...     % pt 16
    -1, 0, 0;...    % pt 17
    0, 0, -1;...    % pt 18
    0, -1, 0;...    % pt 19
    0, 0, -1;...     % pt 20
    1, 0, 0;...     % pt 21
    ]';

% Define faces as a list of vertices numbered above
faces = [...
    1, 2, 3, 4, 1;... % NE arm
    4, 5, 6, 7, 4;... % other arms
    7, 8, 9, 10, 7;...
    10, 11, 12, 1, 10;...
    1, 4, 7, 10, 1;...  % centre
    13, 14, 15, 13, 13;... % top
    15, 16, 17, 15, 15;...
    17, 18, 19, 17, 17;...
    19, 20, 21, 19,19;...
    ];

face_colors = [...
    myred;...       % NE arm
    mygray;...   % other arms
    mygray;...
    mygray;...
    mygray;...   % centre
    myred;...   % top
    myblack;...
    mygray;...
    myblack;...
    ];

end