% [nComponents,sizes,members] = network_components(A)
%
% Daniel Larremore
% April 24, 2014
% larremor@hsph.harvard.edu
% http://danlarremore.com
% Comments and suggestions always welcome.
% 
% INPUTS:
% A                     Matrix. This function takes as an input a 
% network adjacency matrix A, for a network that is undirected. If you
% provide a network that is directed, this code is going to make it
% undirected before continuing. Since link weights will not affect
% component sizes, weighted and unweighted networks work equally well. You
% may provide a "full" or a "sparse" matrix.
%
% OUTPUTS:
% nComponents             INT - The number of components in the network.
% sizes                 vector<INT> - a vector of component sizes, sorted, 
%   descending.
% members               cell<vector<INT>> a cell array of vectors, each
%   entry of which is a membership list for that component, sorted, 
%   descending by component size.
%
% Example: (uncomment and copy and paste into MATLAB command window)
% % Generate a 1000 node network adjacency matrix, A
% A = floor(1.0015*rand(1000,1000)); A=A+A'; A(A==2)=1; A(1:1001:end) = 0;
% % Call networkComponents function
% [nComponents,sizes,members] = networkComponents(A);
% % get the size of the largest component
% sizeLC = sizes(1);
% % get a network adjacency matrix for ONLY the largest component
% LC = A(members{1},members{1});

function [nComponents,sizes,members] = network_components(A)
% Number of nodes
N = size(A,1);
% Remove diagonals
A(1:N+1:end) = 0;
% make symmetric, just in case it isn't
A=A+A';
% Have we visited a particular node yet?
isDiscovered = zeros(N,1);
% Empty members cell
members = {};
% check every node
for n=1:N
    if ~isDiscovered(n)
        % started a new group so add it to members
        members{end+1} = n;
        % account for discovering n
        isDiscovered(n) = 1;
        % set the ptr to 1
        ptr = 1;
        while (ptr <= length(members{end}))
            % find neighbors
            nbrs = find(A(:,members{end}(ptr)));
            % here are the neighbors that are undiscovered
            newNbrs = nbrs(isDiscovered(nbrs)==0);
            % we can now mark them as discovered
            isDiscovered(newNbrs) = 1;
            % add them to member list
            members{end}(end+1:end+length(newNbrs)) = newNbrs;
            % increment ptr so we check the next member of this component
            ptr = ptr+1;
        end
    end
end
% number of components
nComponents = length(members);
for n=1:nComponents
    % compute sizes of components
    sizes(n) = length(members{n});
end

[sizes,idx] = sort(sizes,'descend');
members = members(idx);

end