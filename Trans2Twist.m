function [twist, theta] = Trans2Twist(T)

R = T(1:3, 1:3);
p = T(1:3, 4);

[omega, theta] = Rot2so3(R);
if isequal(omega, zeros(3,1))
    theta = norm(p);
    if theta == 0
        v = zeros(3,1);
    else
        v = p/theta;
    end
else
    A = (eye(3) - exposkew(omega,theta)) * skew(omega) + omega*omega'*theta;
    v = A\p;
end

twist = [v;omega];

end