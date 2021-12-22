function tauVec = InverseDynamics(thetaVec, dthetaVec, ddthetaVec, g,...
                    Ftip, Mlist, Glist, Slist)

%This function uses forward-backward Newton-Euler iterations to solve the 
% equation:
% taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
%           + g(thetalist) + Jtr(thetalist) * Ftip
% Takes thetaVec: n-vector of joint variables,
%       dthetaVec: n-vector of joint rates,
%       ddthetaVec: n-vector of joint accelerations,
%       g: Gravity vector g,
%       Ftip: Spatial force applied by the end-effector expressed in frame 
%             {n+1},
%       Mlist: List of link frames {i} relative to {i-1} at the home 
%              position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns.
% Returns taulist: The n-vector of required joint forces/torques.
% 

n = size(thetaVec, 1);  %degree of freedom of the manipulator
Ai = zeros(6, n);        %Initialize A matrix in which each column is the screw axis of joint i expressed in body inertial frame{i}
AdTi = zeros(6, 6, n + 1); %Initialize the 3D adjoint maxtrix in which each layer is the ajoint matrix of transformation Ti,i-1
Mi = eye(4); % Initialize Mi matix to identical matix for multiplication
Vi = zeros(6, n + 1); %Initialize the body velocity matrix in which each column is the body velocity wrt body inertial frame {i}
Vdi = zeros(6, n + 1); %Initialize the body acceleration matrix in which each column is the body acceleration wrt body inertial frame {i}
Vdi(1: 3, 1) = -g; %denote the body frame {0} (base frame) acceleration with gravity
AdTi(:, :, n + 1) = Trans2Invadjoint(Mlist(:, :, n + 1)); %denote the ajoint matrix of last body frame {n} and the tool frame {n+1} Tn+1,n which is fixed during the movement
Fi = Ftip; % initialize the wrench on the tool frame as the wrench applied to the environment by the end-effector
tauVec = zeros(n, 1); %initialize all joint torques to zero
for i=1: n    
    Mi = Mi * Mlist(:, :, i); % Mutiply the configuration matrix of body inertial frame {i} in order to let it presented in base frame {0}
    Ai(:, i) = Trans2Invadjoint(Mi) * Slist(:, i); %transform the screw axis to body frame
    AdTi(:, :, i) = Trans2adjoint(Twist2Trans(Ai(:, i), -thetaVec(i)) * TransInv(Mlist(:, :, i))); %calulate inv adjoint matrix of Ti-1,i which is the adjoint maxtrix of Ti,i-1 
    Vi(:, i + 1) = AdTi(:, :, i) * Vi(:, i) + Ai(:, i) * dthetaVec(i);
    Vdi(:, i + 1) = AdTi(:, :, i) * Vdi(:, i) ...
                    + Ai(:, i) * ddthetaVec(i) ...
                    + ad(Vi(:, i + 1)) * Ai(:, i) * dthetaVec(i);    
end

for i = n: -1: 1
    Fi = AdTi(:, :, i + 1)' * Fi + Glist(:, :, i) * Vdi(:, i + 1) ...
         - ad(Vi(:, i + 1))' * (Glist(:, :, i) * Vi(:, i + 1));
    tauVec(i) = Fi' * Ai(:, i);
end
                
                
end