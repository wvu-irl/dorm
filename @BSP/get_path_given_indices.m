function [x,u] = get_path_given_indices(obj, G, z)
% function [x,u] = get_path_given_indices(obj, G, z)
% Return sequence of states and control inputs for path computed in the
% online phase. Must run the online phase prior to running this function.
%
% Inptuts:
% G = graph objective after running the online phase, which may be 
%     accessed with bsp.G_ where bsp = BSP(...).
% z = indices of the vertices along the path, which may be computed with
%     get_path_given_goal
%
% Outputs:
% x = sequence of means along the trajectory from the start state to the
%     goal state
% u = sequence of control inputs for edges along the path from the start
%     state to the goal state
% 
% Authors: Jared Strader
%

x=[];
u=[];
for idx=1:length(z)-1
    tempi = z(idx);
    tempj = find(G(tempi).n == z(idx+1));
    tempx=G(tempi).x_seq(tempj).x;
    tempu=G(tempi).u_seq(tempj).u;
    
    for t=1:size(tempx,2)-1
        xt=tempx(:,t);
        ut=tempu(:,t);
        x = [x xt];
        u = [u ut];
        
        if(idx==length(z)-1 && t==size(tempx,2)-1)
            tempi = z(idx+1);
            xt=G(tempi).x;
            x = [x xt];
        end
    end
end

end

