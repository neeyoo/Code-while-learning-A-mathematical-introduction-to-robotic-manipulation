function T = FKtwist(twistlist, thetalist, Initconfig)

%This function is for the calculation of forward kinematics of the
%manipulator for the final tool frame configuration
%The input of this function are the list of joint's twists as well as the
%joint variables.

n = size(twistlist, 2);
T = Initconfig;


for i = 1:n
   T = Twist2Trans(twistlist(:, n+1-i), thetalist(n+1-i)) * T;    
end

end