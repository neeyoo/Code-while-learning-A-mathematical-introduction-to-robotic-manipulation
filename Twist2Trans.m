function T = Twist2Trans(twist, theta)
%this function is for transforming twist coordinates ξ to homogeneous
%transformation matrix

v = twist(1:3);
omega = twist(4:6);

T = [exposkew(omega, theta), (eye(3) - exposkew(omega, theta))*cross(omega,v)+omega*omega'*v*theta; 0 0 0 1];



end