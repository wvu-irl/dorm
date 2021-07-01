function [x_nearest] = nearest(obj, x, G)
%function [x_nearest] = nearest(obj, x_rand, G)
%   Finds nearest point to x_rand in graph G where x_rand = [x;y] and
%   G(i).x = [x_i;y_i] for vertex i in the tree
%
%   TODO: update to use n_configurations instead of just first 2 elements
%         of state
%
%   Authors: Jared Strader

dmin = inf;
for i=1:length(G)
    dtmp = norm( G(i).x(1:2) - x(1:2) );
    if(dtmp < dmin)
        dmin = dtmp;
        imin = i;
    end
end

% x_nearest = G(imin).x;
% x_index = imin;

x_nearest = G(imin);

end

