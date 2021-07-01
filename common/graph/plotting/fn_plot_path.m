function [nu] = fn_plot_path(x_goal,G,col)

xT=[]; %final state
x0=[]; %initial state

%index of goal vertex
minval=inf;
minidx=-1;
for i=1:length(G)
    d = norm(G(i).x(1:2) - x_goal(1:2));
    if(d<minval)
        minval=d;
        minidx=i;
        xT = G(minidx).x;
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
    
    x = G(u).x;
    y = G(uprev).x;
    x0 = y;
    
    hold on;
    line([x(1), y(1)], [x(2), y(2)], 'Linewidth', 2, 'Color', col);
    u=uprev;
end

hold on;
plot(x0(1),x0(2),'bo','MarkerFaceColor','g','MarkerSize',10);
hold on;
plot(xT(1),xT(2),'bo','MarkerFaceColor','r','MarkerSize',10);

end

