function Jacob = JacobianBody(twistlist, thetalist, Initconfig)

%This function is for the calculation of body manipulator Jacobian where
%the end-effector velocities are with respect to the tool frame.
%The input of this function are the list of joint's twists as well as the
%joint variables.

n = size(twistlist, 2);
Jacob = twistlist;
T = Initconfig;


for i = n:-1:1
   T = Twist2Trans(twistlist(:, i), thetalist(i)) * T;
   Jacob(:,i) = Trans2Invadjoint(T)*twistlist(:,i);
    
end

end