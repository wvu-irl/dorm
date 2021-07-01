function [] = plot_rects(rects, color)
% function [] = plot_rects(rects, color)
% This function plots the rectangles using the specified color on the
% current figure.
%
% Inputs:
% rects = vector of rectangles, see "generate_rect" function for details
% color = [r_value, g_value, b_value] where values are in the range [0,1]
%
% Authors: Jared Strader

if(nargin<2 || isempty(color))
    color=[0.3010 0.7450 0.9330];
end

for i=1:length(rects)
    hold on;
    rectangle('Position',rects(i).rect,'FaceColor',color,'EdgeColor','k');
%     rectangle('Position',rects(i).rect,'FaceColor',,'EdgeColor','k');
end

end


