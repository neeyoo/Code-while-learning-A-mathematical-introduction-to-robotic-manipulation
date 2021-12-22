# Code-while-learning-A-mathematical-introduction-to-robotic-manipulation
Some code of screw theory in Matlab


1. `omega_skew = skew(omega)`   
This function is for calculation the skew symmetric matrix 

2. `R = exposkew(omega, theta)`  
This function is for creating the rotational matrix by using Rodrigues' formula

3. `T = Twist2Trans(twist, theta)`  
This function is for transforming twist coordinates ξ = [v;ω] to homogeneous transformation matrix

4. `Tinv = TransInv(T)`  
This function is for calculating the inverse transformation matrix in SE(3)

5. `T = FKtwist(twistlist, thetalist, Initconfig)`  
This function is for the calculation of forward kinematics of the manipulator for the final tool frame configuration with twist coordinate inputs

6. `Adg = Trans2adjoint(Trans)`  
This function is for calculating the adjoint transformation which is a 6*6 matrix which transforms twists from one coordinate frame to another.

7. `AdgInv = Trans2Invadjoint(Trans)`  
This function is for calculating the inverse adjoint transformation which is a 6*6 matrix which transforms twists from one coordinate frame to another.

8. `Jacob = JacobianSpatial(twistlist, thetalist)`
This function is for the calculation of spatial manipulator Jacobian

9. `Jacob = JacobianBody(twistlist, thetalist, Initconfig)`  
This function is for the calculation of body manipulator Jacobian where the end-effector velocities are with respect to the tool frame.

10. `tauVec = InverseDynamics(thetaVec, dthetaVec, ddthetaVec, g, Ftip, Mlist, Glist, Slist)`  
This function uses forward-backward Newton-Euler iterations to solve the inverse dynamics

