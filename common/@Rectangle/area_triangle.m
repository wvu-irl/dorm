function A = area_triangle(p1, p2, p3)
%function A = area_triangle(p1, p2, p3)
% Returns area of a triangle drawn from points p1, p2, and p3 using the 
% shoelace formula
%
% Inputs:
% Note the order doesn't matter.
% p1, p2, and p3 = [x;y] 
%
% Authors: Jared Strader

M = [p1(1), p2(1), p3(1);...
     p1(2), p2(2), p3(2);...
     1    , 1    , 1];
A = 1/2*abs(det(M));

end

