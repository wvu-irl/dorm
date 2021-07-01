function [z,success] = fn_quick_steer(x,y,nu)

z(1,1) = x(1);
z(2,1) = x(2);

z(1,2) = x(1) + ((y(1)-x(1))*nu)/norm(y-x);
z(2,2) = x(2) + ((y(2)-x(2))*nu)/norm(y-x);

end

