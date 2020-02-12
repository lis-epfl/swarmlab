function [dist_matrix] = pos2dist(positions)
% pos2dist - Compute the relative distances matrix from the positions 
%            matrix
%
% Inputs:
%   positions - matrix of size (3,nb_agents)
%
% Ouputs:
%   dist_matrix - symmetric matrix with pairwise distances
%
%
    distances = pdist(positions');
    dist_matrix = squareform(distances);

end