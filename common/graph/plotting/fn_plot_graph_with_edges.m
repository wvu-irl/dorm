function [] = fn_plot_graph(T)
%function [] = fn_plot_graph(T)
%   Plots the tree T see example code for structure of T
%
%   Authors: Jared Strader

% for i=2:length(T)
%     x1 = T(i).x;
%     hold on;
%     for j=1:length(T(i).n)
%         x2 = T(T(i).n(j)).x;
%         line([x1(1), x2(1)], [x1(2), x2(2)], 'Color', 'k');
%     end
% end

X=[];
Y=[];
for i=2:length(T)
    x1 = T(i).x;
    for j=1:length(T(i).n)
        x2 = T(T(i).n(j)).x;
        X = [X, x1(1), x2(1), nan];
        Y = [Y, x1(2), x2(2), nan];
    end
end
plot(X, Y, 'Color', 'k');
alpha(0.5)
% plot(X, Y, 'Color', 'k');

for i=2:length(T)
    hold on;
    x1 = T(i).x;
    plot(x1(1),x1(2),'k.');
end
% plot(T(1).x(1),T(1).x(2),'bo'); hold on;

end

