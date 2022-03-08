function C = VelQuadraticForces(thetaVec, dthetaVec, Mlist, Glist, Slist)
% This function is for calculating the coriolis and centrifugal forces

% The idea of the calculation is to use inversedynamics function and let
% the joint acceleration and gravity field equal to zero.

C = InverseDynamics(thetaVec, dthetaVec, zeros(length(thetaVec), 1), zeros(3,1), zeros(6,1), Mlist, Glist, Slist);


end