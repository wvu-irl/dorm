function [path_indices] = fn_get_path(x_goal,G)

path_indices=[];

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

%plot path from goal to start
u = minidx;
path_indices=u;
while ~isempty(u)
    uprev = G(u).parent;
    if(isempty(uprev))
        break;
    end
    
    x = G(u).x;
    y = G(uprev).x;
    
    u=uprev;
    path_indices=[uprev path_indices];
end

end

