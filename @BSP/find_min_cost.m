function [n] = find_min_cost(obj, q, G)
%q = list of graph indices in q
%G = graph

min_val=inf;
min_idx=-1;
for i=1:length(q)
    if(G(q(i)).c < min_val)
        min_val = G(q(i)).c;
        min_idx = i;
    end
end

if(min_idx==-1)
    error('error! this should not happpen.');
end

n=min_idx;

end

