function Jacob = JacobianSpatial(twistlist, thetalist)

%This function is for the calculation of spatial manipulator Jacobian
%The input of this function are the list of joint's twists as well as the
%joint variables.

n = size(twistlist, 2);
Jacob = twistlist;
T = eye(4);


for i = 2:n
   T = T * Twist2Trans(twistlist(:, i-1), thetalist(i-1));
   Jacob(:,i) = Trans2adjoint(T)*twistlist(:,i);
    
end

end