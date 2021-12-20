function Jacob = JacobianBody(twistlist, thetalist, Initconfig)

%This function is for the calculation of body manipulator Jacobian where
%the end-effector velocities are with respect to the tool frame.
%The input of this function are the list of joint's twists as well as the
%joint variables.

n = size(twistlist, 2);
Jacob = twistlist;
T = Initconfig;


for i = 1:n
   T = Twist2Trans(twistlist(:, n+1-i), thetalist(n+1-i)) * T;
   Jacob(:,n+1-i) = Trans2Invadjoint(T)*twistlist(:,n+1-i);
    
end

end