function [] = fn_plot_points(P)
%function [] = fn_plot_points(P)
%   Plots points P where P is an 2xN vector
%
%   Authors: Jared Strader

for i=1:size(P,2)
    hold on;
    plot(P(1,i), P(2,i), 'co');
end

end