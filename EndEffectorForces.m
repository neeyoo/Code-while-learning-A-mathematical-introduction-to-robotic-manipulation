function tauFtip = EndEffectorForces(thetaVec, Ftip, Mlist, Glist, Slist)
% This function is for calculating the torque applied on the joint which is
% due to a wrench applied on the endeffector, this wrench is expressed in
% endeffector frame

% The idea for the calculation is by using inverdynamics function and let
% the joint velocity, acceleration and gravity field equal to zeros.

n = length(thetaVec);
tauFtip = InverseDynamics(thetaVec, zeros(n, 1), zeros(n, 1), ...
                         zeros(3,1), Ftip, Mlist, Glist, Slist);


end