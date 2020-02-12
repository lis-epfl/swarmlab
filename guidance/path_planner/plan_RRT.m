function path_out = plan_RRT(wpt_start, wpt_end, map)

% PLAN_RRT - create a path from a start node to an end node
% using the RRT (Rapidly-exploring Random Tree) algorithm 
%
% Inputs:
%   wpt_start   - starting wp
%   wpt_end     - ending wp
%   map         - map parameters
%
% Outputs:
%   path_out    - output path


    % Standard length of path segments
    segment_length = 100;

    % Desired down position is down position of end node
    pd = wpt_end(3);
    chi = -9999;
    
    % Specify start and end nodes from wpt_start and wpt_end
    start_node = [wpt_start(1), wpt_start(2), pd, chi, 0, 0, 0];
    end_node = [wpt_end(1), wpt_end(2), pd, chi, 0, 0, 0];
    % format:  [pn, pe, pd, chi, cost, parent_idx, flag_connect_to_goal]

    % Establish tree starting with the start node
    tree = start_node;
    
    % Check to see if start_node connects directly to end_node
    if ( (norm(start_node(1:3)-end_node(1:3)) < segment_length )...
            & (collision(start_node,end_node,map)==0) )
        path = [start_node; end_node];
    else
        nb_paths = 0;
        while nb_paths < 3
            [tree,flag] = extend_tree(tree,end_node,segment_length,map,pd,chi);
            nb_paths = nb_paths + flag;
        end
    end

    % Find path with minimum cost to end_node
    path = find_minimum_path(tree,end_node);
    path_out = smooth_path(path,map);
    plot_map(map,path,path_out,tree);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate Random Node -
%   create a random node (initialize)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function node = generate_random_node(map,pd,chi)

    % randomly pick configuration
    pn       = map.width*rand;
    pe       = map.width*rand;
    pd       = pd; % constant altitute paths
    cost     = 0;
    node     = [pn, pe, pd, chi, cost, 0, 0];
    % format:  [N, E, D, chi, cost, parent_idx, flag_connect_to_goal]
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Collision -
%   check to see if a node is in collsion with obstacles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function collision_flag = collision(start_node, end_node, map)

    collision_flag = 0;
    
    [X,Y,Z] = points_along_path(start_node, end_node, 0.1);

    for i = 1:length(X)
        if Z(i) >= down_at_NE(map, X(i), Y(i))
            collision_flag = 1;
        end
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Points Along Path -
%   Find points along straight-line path separted by Del (to be used in
%   collision detection)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X,Y,Z] = points_along_path(start_node, end_node, step_length)

    X = [start_node(1)];
    Y = [start_node(2)];
    Z = [start_node(3)];
    
    q = [end_node(1:3)-start_node(1:3)];
    L = norm(q);
    q = q/L;
    
    w = start_node(1:3);
    for i=2:floor(L/step_length)
        w = w + step_length*q;
        X = [X, w(1)];
        Y = [Y, w(2)];
        Z = [Z, w(3)];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Down At NE -
%   find the world down coordinate at a specified (n,e) location
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function down = down_at_NE(map, n, e)

      [d_n,idx_n] = min(abs(n - map.buildings_north));
      [d_e,idx_e] = min(abs(e - map.buildings_east));

      if (d_n<=map.building_width) && (d_e<=map.building_width)
          down = -map.buildings_heights(idx_e);
      else
          down = 0;
      end

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Extend Tree -
%   extend tree by randomly selecting point and growing tree toward that
%   point
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [new_tree,flag] = extend_tree(tree,end_node,segment_length,map,pd,chi)

  flag1 = 0;
  while flag1==0
    % Select a random point
    random_node = generate_random_node(map,pd,chi);
    
    % Find leaf on node that is closest to randomPoint
    tmp = tree(:,1:3)-ones(size(tree,1),1)*random_node(1:3);
    [dist,idx] = min(diag(tmp*tmp'));
    L = min(sqrt(dist), segment_length); 
    cost     = tree(idx,5) + L;
    tmp = random_node(1:3)-tree(idx,1:3);
    new_point = tree(idx,1:3)+L*(tmp/norm(tmp));
    new_node = [new_point, chi, cost, idx, 0]; 

    if collision(tree(idx,:), new_node, map)==0
      new_tree = [tree; new_node];
      flag1=1;
    end
  end
  
  % Check to see if new node connects directly to end_node
  if ( (norm(new_node(1:3)-end_node(1:3))<segment_length )...
      &&(collision(new_node,end_node,map)==0) )
    flag = 1;
    new_tree(end,7)=1;  % mark node as connecting to end.
  else
    flag = 0;
  end
  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find Minimum Path -
%   find the lowest cost path to the end node
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function path = find_minimum_path(tree,end_node)
    
    % Find nodes that connect to end_node
    connecting_nodes = [];
    for i=1:size(tree,1)
        if tree(i,7)==1
            connecting_nodes = [connecting_nodes; tree(i,:)];
        end
    end

    % Find minimum cost last node
    [tmp,idx] = min(connecting_nodes(:,5));

    
    % Construct lowest cost path
    path = [connecting_nodes(idx,:); end_node];
    parent_node = connecting_nodes(idx,6);
    while parent_node>1
        parent_node = tree(parent_node,6);
        path = [tree(parent_node,:); path];
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% smoothPath -
%   smooth the waypoint path 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function new_path = smooth_path(path,map)

    new_path = path(1,:); % add the start node 
    ptr =2;  % pointer into the path
    while ptr <= size(path,1)-1
        if collision(new_path(end,:), path(ptr+1,:), map)~=0 % if there is a collision
            new_path = [new_path; path(ptr,:)];  % add previous node
        end
        ptr=ptr+1;
    end
    new_path = [new_path; path(end,:)];

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Map -
%   plot obstacles and path
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_map(map,path,smoothed_path,tree)
  
    % Draw tree
    for i=2:size(tree,1)
        X = [tree(i,1), tree(tree(i,6),1)];
        Y = [tree(i,2), tree(tree(i,6),2)];   
        Z = [tree(i,3), tree(tree(i,6),3)];            
        plot3(Y,X,-Z,'g')
    end
  
    % Draw path
    X = path(:,1);
    Y = path(:,2);
    Z = path(:,3);
    plot3(Y,X,-Z,'r','linewidth',2);

    % Draw smooth path
    X = smoothed_path(:,1);
    Y = smoothed_path(:,2);
    Z = smoothed_path(:,3);
    plot3(Y,X,-Z,'k','linewidth',2);

end