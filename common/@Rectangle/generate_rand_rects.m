function rects = generate_rand_rects(xspace, yspace, n, side)
%function rects = generate_rand_rects(xspace, yspace, n, side)
%   This functions is used to generate a set of rectangles which may
%   represent obstacles, gps regions, etc... (the rectangles are assumed 
%   to be axis aligned)
%
%   Inputs: xspace = [xmin xmax] where the center of the rectangles
%                                will be generated within this range for x
%           yspace = [ymin ymax] where the center of the rectangles
%                                will be generated within this range for y
%           n = number of rectangles
%           side = [side_min, side_max] where the rectangles width and
%                                       height will be generated within 
%                                       this range
%
%   Example:
%	rectangles = fn_generate_rects([-10,10],[-10,10],10,[1,3])
%
%	Authors: Jared Strader

%generate obstacles
rects=[];
for i=1:n
    new_xspace=[xspace(1) + (side(2)-side(1))/2,...
                xspace(2) - (side(2)-side(1))/2];
    new_yspace=[yspace(1) + (side(2)-side(1))/2,...
                yspace(2) - (side(2)-side(1))/2];;
    x = xspace(1) + rand*(new_xspace(2)-new_xspace(1));
    y = yspace(1) + rand*(new_yspace(2)-new_yspace(1));
    w = side(1) + rand*(side(2)-side(1));
    h = side(1) + rand*(side(2)-side(1));
    rects(i).area = w*h;
    rects(i).points = [x-w/2, x+w/2, x-w/2, x+w/2;...
                       y+h/2, y+h/2, y-h/2, y-h/2];
    rects(i).rect = [x-w/2,y-h/2,w,h];
end

%plot rectangles (just for testing)
% figure;
% axis([xspace(1) - side(2),...
%       xspace(2) + side(2),...
%       yspace(1) - side(2),...
%       yspace(2) + side(2)]);
% for i=1:n
%     hold on;
%     rectangle('Position',obstacles(i).rect);
% end

end

