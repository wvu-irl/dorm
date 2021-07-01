function [X,indices] = fn_near(T,x,nu)
%function [X] = fn_near(T,x,nu)
%   Returns all vertices within a ball radius nu of the vertex x
%
%   Authors: Jared Strader

X=[];
indices=[];
for i=1:length(T)
    if(norm( T(i).x(1:2) - x(1:2)) < nu) 
        X = [X, T(i)];
        indices=[indices, i];
    end
end

end

