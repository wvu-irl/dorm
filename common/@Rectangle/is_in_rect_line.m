function is_in_rect = is_in_rect_line(xi, xf, rects)
% is_in_rect = is_in_rect_line(xi, xf, rects)
% Return true if straight line path from xi to xf interects any rectangle
% in rects
%
% Inputs:
% xi = [x;y], initial point of line segment
% xf = [x;y], final point of line segment
% rects = vector of rectangles, see "generate_rect" function for details
%
% Authors: Jared Strader

if(isempty(rects))
    is_in_rect = false;
    return;
end

is_in_rect = false;
for i=1:length(rects)
    ul = rects(i).points(:,1);
    ur = rects(i).points(:,2);
    bl = rects(i).points(:,3);
    br = rects(i).points(:,4);

    l1 = ~Rectangle.is_line_segments_intersecting(xi,xf,ul,ur);
    l2 = ~Rectangle.is_line_segments_intersecting(xi,xf,ur,br);
    l3 = ~Rectangle.is_line_segments_intersecting(xi,xf,br,bl);
    l4 = ~Rectangle.is_line_segments_intersecting(xi,xf,bl,ul);
    
    is_in_rect = ~(l1 & l2 & l3 & l4);
    
    if(is_in_rect)
        break;
    end
end



end

