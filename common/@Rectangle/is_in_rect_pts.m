function is_in_rect = is_in_rect_pts(pts, rects)
% function is_in_rect = is_in_rect_pts(pts, rects)
% This function check if any point in the vector pts are contained in 
% a rectangle (rectangles may be rotated or not)
%
% Inputs:
% pts = 2xN vector where pts[:,i] = [x;y]
% rects = vector of rectangles, see "generate_rect" function for details
%
% Authors: Jared Strader

is_in_rect=false;
for i=1:size(pts,2)
    xpos(1) = pts(1,i);
    xpos(2) = pts(2,i);
    for j=1:length(rects)
        A(1) = Rectangle.area_triangle(xpos,...
                                       rects(j).points(:,1),...
                                       rects(j).points(:,2));
        A(2) = Rectangle.area_triangle(xpos,...
                                       rects(j).points(:,2),...
                                       rects(j).points(:,3));
        A(3) = Rectangle.area_triangle(xpos,...
                                       rects(j).points(:,3),...
                                       rects(j).points(:,4));
        A(4) = Rectangle.area_triangle(xpos,...
                                       rects(j).points(:,1),...
                                       rects(j).points(:,4));
                            
        %sum > rect: point outside rect, any A == 0: point lies on 
        %rectangle (unlikely with real numbers so ignored), otherwise: 
        %point is inside rectangle
        if(A(1)+A(2)+A(3)+A(4) < rects(j).area)
            is_in_rect = true;
            break;
        end
    end
    if(is_in_rect==true)
        break;
    end
end

end

