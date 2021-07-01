function [is_intersecting] = is_line_segments_intersecting(a,b,c,d)
% function [is_intersecting] = is_line_segments_intersecting(a,b,c,d)
% Check if line segment going through points a and b intersects line 
% segment going through points c and d. 
%
% Returns true if line segments intersect and false if line segments do 
% not intersect.
%
% Authors: Jared Strader

is_intersecting = Rectangle.ccw(a,c,d) ~= Rectangle.ccw(b,c,d) & Rectangle.ccw(a,b,c) ~= Rectangle.ccw(a,b,d);

end

