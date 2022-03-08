function ddthetaVec = ForwardDynamics(thetaVec, dthetaVec, tauVec, g,...
    Ftip, Mlist, Glist, Slist)
 %This function is for the calculation of forwoard dynamics of the
 %manipulator

ddthetaVec = MassMatrix(thetaVec, Mlist, Glist, Slist)\...
    (tauVec - VelQuadraticForces(thetaVec, dthetaVec, Mlist, Glist, Slist)...
    -GravityForce(thetaVec, g, Mlist, Glist, Slist)...
    - EndEffectorForces(thetaVec, Ftip, Mlist, Glist, Slist));

end