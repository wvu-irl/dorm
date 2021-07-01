function [X,indices] = near(obj, G, x, nu)
%function [X] = fn_near(T,x,nu)
%   Returns all vertices within a ball radius nu of the vertex x
%
%   TODO: update to use n_configurations instead of just first 2 elements
%         of state
%
%   Authors: Jared Strader

X=[];
indices=[];
for i=1:length(G)
    if(norm( G(i).x(1:2) - x(1:2)) < nu) 
        X = [X, G(i)];
        indices=[indices, i];
    end
end

end

