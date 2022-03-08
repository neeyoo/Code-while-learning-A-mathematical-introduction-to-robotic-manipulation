function [omega, theta] = Rot2so3(R)

% Takes R (rotation matrix).
% Returns the corresponding so(3) representation of exponential 
% coordinates: omega and theta.


acosinput = (trace(R) - 1) / 2;
if acosinput >= 1
    omega = zeros(3,1);
    theta = 0;
elseif acosinput <= -1
    if ~NearZero(1 + R(3, 3))
        omega = (1 / sqrt(2 * (1 + R(3, 3)))) ...
              * [R(1, 3); R(2, 3); 1 + R(3, 3)];
    elseif ~NearZero(1 + R(2, 2))
        omega = (1 / sqrt(2 * (1 + R(2, 2)))) ...
              * [R(1, 2); 1 + R(2, 2); R(3, 2)];
    else
        omega = (1 / sqrt(2 * (1 + R(1, 1)))) ...
              * [1 + R(1, 1); R(2, 1); R(3, 1)];
    end
    theta = pi;
else
	theta = acos(acosinput);
    omega = (1/(2 * sin(theta))) * [R(3,2) - R(2,3); R(1,3) - R(3,1); R(2,1) - R(1,2)];
end
end