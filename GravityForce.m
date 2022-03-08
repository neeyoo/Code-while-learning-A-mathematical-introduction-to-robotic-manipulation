function G = GravityForce(thetaVec, g, Mlist, Glist, Slist)
% This function is for calculating the gravitational force of the
% manipulator.

% The idea of the calculation is to use inversedynamics function and let
% the joint acceleration and joint velocity equal to zero.
n = length(thetaVec);
G = InverseDynamics(thetaVec, zeros(n, 1), zeros(n, 1), g,...
                    zeros(6, 1), Mlist, Glist, Slist);

end