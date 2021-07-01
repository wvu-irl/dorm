classdef Rectangle < handle
    properties (Abstract, Constant)
        % No properties...
    end
    
    methods (Static)
        %% user interface
        %generate rectangles
        rects = generate_rand_rects(xspace, yspace, n, side)
        rect = generate_rect(x, y, w, h)
        
        %check if line is intersecting any rectangle, returns bool
        is_in_rect = is_in_rect_line(xi, xf, rects)
        
        %check if a point is inside any rectangle, returns bool
        is_in_rect = is_in_rect_pts(pts, rects)
        
        %plot rectangles
        [] = plot_rects(rects, color)
        
        %% helper functions
        A = area_triangle(p1, p2, p3)
        is_ccw = ccw(p1, p2, p3)
        is_intersecting = is_line_segments_intersecting(a, b, c, d)
        
    end
    
end