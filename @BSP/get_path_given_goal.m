function [path_indices] = get_path_given_goal(obj, G, x_goal)
% function [path_indices] = get_path_given_goal(obj, G, x_goal)
% Return indices of indices of vertices along path from start state to
% goal state where the start state is used as input to the online phase.
% Must run the online phase prior to running this function.
% 
% Inputs:
% G = graph objective after running the online phase, which may be 
%     accessed with bsp.G_ where bsp = BSP(...).
% x_goal = desired goal state, the vertex closest to x_goal will be used
%          as the goal vertex for return the path to goal
%
% Outputs:
% path_indices = vertices from start state to goal state where 
% path_indices[i] is the ith vertex along the path in the graph. Must
% run online phase for path to exist.
%
% Authors: Jared Strader
%

if(isempty(G))
    error('Error! Graph is empty.');
end

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

