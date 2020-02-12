function M =  compute_neighborhood(distance_matrix, r_comm, max_neig)
    % compute_neighborhood - compute the adjacency matrix for a given graph
    %
    % Inputs:
    %   distance_matrix - matrix of distances, symmetric
    %   r_comm - radius within which an agents is considered a neighbor
    %   max_neig - maximum number of neighbors
    %
    % Outputs:
    %   M - neighborhood matrix, can be non-symmetric
    %
    
    %% Build adjacency matrix
    % Adjacency matrix M(i,j) (asymmetric in case of limited fov): 
    % column j contains the neighbours i of agent j
    % Note: agent j is not neig of itself (zeros on diagonal)
    [~,N]   = size(distance_matrix);
    M       = ones(N,N)-eye(N,N);
    to_cut  = eye(N,N);
    to_cut  = to_cut(:);
    distance_matrix(to_cut==1) = Inf;
    
    % Constraint on neighborhood given by the limited radius of
    % communication
    M(distance_matrix > r_comm) = 0;
    % Number of neighbours (i.e. agents or obstacles)
    nn = sum(M,1);

    % Constraint on neighborhood given by the topological distance
    to_cut  = find(nn > max_neig);
    if ~isempty(to_cut)
        [~,I]   = sort(distance_matrix(:,to_cut), 1);
        for agent = 1:length(to_cut)
            M(I((max_neig+1):end,agent),agent) = 0; 
        end
    end

end
