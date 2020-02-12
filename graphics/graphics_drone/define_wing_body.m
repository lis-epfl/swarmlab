function [V,F,facecolors] = defineFixedBody()

% DEFINEBODY - Defines the vertices and faces for a fixed wing
%
% Syntax: defineFixedBody()
%
% Outputs:
%   V          - vertices
%   F          - faces
%   facecolors - colors for drone faces


% Define colors
myred    = [1, 0, 0];
mygreen  = [0, 1, 0];
% myblue   = [0, 0, 1];
% myyellow = [1, 1, 0];
% mycyan   = [0, 1, 1];
myblack  = [0, 0, 0];
% mygray  = [0.7, 0.7, 0.7];

% Define the physical location of vertices
V = [...
    2, 0, 0;...     % pt 1
    0, 4, 0;...     % pt 2
    -1, 4, 0;...    % pt 3
    0, 0, 0;...     % pt 4
    -1, -4, 0;...   % pt 5
    0, -4, 0;...    % pt 6
    2, 0, 0;...     % pt 7
    0.4, 0, -1;...  % pt 8
    0, 0, -1;...    % pt 9
    0, 0, 0;...     % pt 10
    ]';

% Define faces as a list of vertices numbered above
F = [...
    1, 6, 5, 4;...  % left wing
    1, 4, 3, 2;...  % right wing
    1, 4, 9, 8;...  % tail
    ];

facecolors = [...
    myred;...     % left wing
    mygreen;...   % right wing
    myblack;...   % tail
    ];

end