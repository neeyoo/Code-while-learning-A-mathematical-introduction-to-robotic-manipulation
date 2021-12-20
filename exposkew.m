function R = exposkew(omega, theta)
%This function is for creating the rotational matrix by using Rodrigues'
%formula

%The inputs are rotational vection 'omega' and rotational angle 'theta'

R = eye(3) + skew(omega) * sin(theta) + skew(omega)^2 * (1 - cos(theta));



end