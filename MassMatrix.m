function M = MassMatrix(thetaVec, Mlist, Glist, Slist)
% This function is for calculation the mass matrix of the manipulator

% The idea is to use inversedynamic function and let the joint velocity and
% gravity field equal to zero and the denote each joint acceleration to 1
% for calculating the columns of mass matrix.

n = length(thetaVec); %degree of freedom
M = zeros(n); %initialize mass matrix
dthetaVec = zeros(n,1);
g = zeros(3,1);
Ftip = zeros(6,1);
for i = 1:n
    ddthetaVec = zeros(n,1);
    ddthetaVec(i) = 1;
    M(:,i) = InverseDynamics(thetaVec, dthetaVec, ddthetaVec, g,...
                    Ftip, Mlist, Glist, Slist);
end


end