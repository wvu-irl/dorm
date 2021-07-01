function [is_ccw] = ccw(p1, p2, p3)
%function [is_ccw] = ccw(p1, p2, p3)
%Check if points p1, p2, and p3 are in counter clockwise order. Returns
%true if points are in ccw order and false if points are not in ccw
%
% Authors: Jared Strader

temp1 = (p3(2)-p1(2))*(p2(1)-p1(1));
temp2 = (p2(2)-p1(2))*(p3(1)-p1(1));
is_ccw = temp1 > temp2;

end

