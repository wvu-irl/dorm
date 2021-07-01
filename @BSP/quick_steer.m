function [z] = quick_steer(obj, x, y, nu)
%function [z] = quick_steer(obj, x, y, nu)
%   TODO: update for higher dimensional configurations
%
%   Authors: Jared Strader

% if(norm(y-x) > nu)
%     z(1,1) = x(1);
%     z(2,1) = x(2);
% 
%     z(1,2) = x(1) + ((y(1)-x(1))*nu)/norm(y-x);
%     z(2,2) = x(2) + ((y(2)-x(2))*nu)/norm(y-x);
% else
%     z(1,1) = x(1);
%     z(2,1) = x(2);
% 
%     z(1,2) = y(1);
%     z(2,2) = y(2);
% end

z(1,1) = x(1);
z(2,1) = x(2);

z(1,2) = x(1) + ((y(1)-x(1))*nu)/norm(y-x);
z(2,2) = x(2) + ((y(2)-x(2))*nu)/norm(y-x);

end

