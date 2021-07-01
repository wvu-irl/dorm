function [nu] = fn_plot_trajectory(x_goal,G,col)

%index of goal vertex
minval=inf;
minidx=-1;
for i=1:length(G)
    d = norm(G(i).x(1:2) - x_goal(1:2));
    if(d<minval)
        minval=d;
        minidx=i;
    end
end
%print cost
nu = G(minidx).nu;

%plot path from goal to start
u = minidx;
while ~isempty(u)
    uprev = G(u).parent;
    if(isempty(uprev))
        break;
    end
    
    tempidx = find(G(uprev).n == u);
    x = [G(uprev).x_seq(tempidx).x];
    x = [x G(u).x];
    hold on;
    plot(x(1,:), x(2,:), 'Linewidth', 2, 'Color', col);
    
%     hold on;
%     line([x(1), y(1)], [x(2), y(2)], 'Linewidth', 2, 'Color', col);
    u=uprev;
end

end

