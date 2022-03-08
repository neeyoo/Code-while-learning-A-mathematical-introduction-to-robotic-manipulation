function T = Twist2Trans(twist, theta)
%this function is for transforming twist coordinates ξ = [v;ω] to homogeneous
%transformation matrix

v = twist(1:3);
omega = twist(4:6);

if isequal(omega,zeros(3,1))
    T = [eye(3) v*theta; 0 0 0 1];
else
    T = [exposkew(omega, theta), (eye(3) - exposkew(omega, theta))*cross(omega,v)+omega*omega'*v*theta; 0 0 0 1];
end


end