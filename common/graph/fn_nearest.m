function [x_nearest] = fn_nearest(x_rand,T)
%function [x_near] = nearest(x_rand,T)
%   Finds nearest point to x_rand in tree T where x_rand = [x;y] and
%   T(i).x = [x_i;y_i] for vertex i in the tree
%
%   Authors: Jared Strader

dmin = inf;
for i=1:length(T)
    dtmp = norm( T(i).x - x_rand );
    if(dtmp < dmin)
        dmin = dtmp;
        imin = i;
    end
end

% x_nearest = T(imin).x;
% x_index = imin;

x_nearest = T(imin);

end

