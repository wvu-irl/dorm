function rect = generate_rect(x, y, w, h)
% function rect = generate_rect(x, y, w, h)
% Generates a rectangle structure.
%
% Inputs:
% x = x of upper left
% y = y of upper left
% w = width
% h = height
%
% Authors: Jared Strader

rect.area = w*h;

rect.points = [x,...   %x ul
               x+w,... %x ur
               x,...   %x bl
               x+w;... %x br
               y,...   %y ul
               y,...   %y ur
               y-h,... %y bl
               y-h];   %y br
                   
rect.rect = [x,...
             y-h,...
             w,...
             h];

end

