clear;clc;
model = loadurdfmodel('universalUR5.urdf.xml');

%% Transfer the paramters to the following form
%       Mlist: List of link frames {i} relative to {i-1} at the home 
%              position,
%       Glist: Spatial inertia matrices Gi of the links,
%       Slist: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns.

% Get the Mlist and Glist, take the first link as base
Mlist = zeros(4,4,length(model.link));
Glist = zeros(6,6,length(model.link)-1);
for i = 1:length(model.link)
    if i == 1
        Mlist( :, :, i) = model.joint{i}.Trans * model.link{i+1}.Trans;
        Glist(:, :, i) = [model.link{i+1}.mass * eye(3) zeros(3); ...
                            zeros(3) model.link{i+1}.inertia];
    elseif i == length(model.link)
        Mlist(:, :, i) = TransInv(model.link{i}.Trans) * model.joint{i}.Trans;
    else
        Mlist(:, :, i) = TransInv(model.link{i}.Trans) * model.joint{i}.Trans...
            * model.link{i+1}.Trans;
        Glist(:, :, i) = [model.link{i+1}.mass * eye(3) zeros(3); ...
                            zeros(3) model.link{i+1}.inertia];
    end   
end

%Get the Slist, the twist coordinates of the joints
Slist = zeros(6, length(model.joint) - 1);
Trans = eye(4);
for i = 1:size(Slist, 2)
   Trans = Trans * model.joint{i}.Trans;
   omega = Trans(1:3, 1:3) * model.joint{i}.Axis';
   v = - cross(omega, Trans(1:3, 4));
   Slist(:, i) = [v; omega];
    
end
TransInit = Trans * model.joint{i+1}.Trans;

%%
thetaVec = [0; -pi/4; pi/4; -pi/2; -pi/2; 0];
dthetaVec = [0; 0; 0; 0; 0; 0];
g = [0; 0; -9.8];
Ftip = [0; 0; 0; 0; 0; 0];
tauVec = [0;0;0;0;0;0];
dt = 0.005;
ax = gca;
M = diag([1 1 1 1 1 1]);
B = diag([50 50 50 5 5 5]);
K = diag([500 500 500 200 200 200]);
Xf = FKtwist(Slist, thetaVec, TransInit);
%Xf(1:3,4) = Xf(1:3,4) - [0; 0; 0.2];
X = zeros(6,1);
Xpre = zeros(6,1);
Xd = zeros(6,1);
Xdpre = zeros(6, 1);
Xdd = zeros(6,1);
ddthetaVec = zeros(6,1);

for T = 0:dt:5
    tauVecInvGra = GravityForce(thetaVec, g, Mlist, Glist, Slist);
    Ttool = FKtwist(Slist, thetaVec, TransInit);
    Jb = JacobianBody(Slist, thetaVec, TransInit);
    [twist, theta] = Trans2Twist(TransInv(Ttool) * Xf); 
    X = -twist * theta; % Xe = log(X^-1 * Xd);
    Xd = Jb * dthetaVec;
    %Xd = (X - Xpre)/dt;
    Xdd = (Xd - Xdpre)/dt;
    tauVecImpe = impedanceCrtl(X, Xd, Xdd*0, M, B, K, Jb);
    tauVec = tauVecInvGra + tauVecImpe;
    Xpre = X;
    Xdpre = Xd;
    
    if T < 0.03
        Ftip = [20;20;20;20;20;20]*10;
    else
        Ftip = zeros(6,1);
    end
    
%    ddthetaVec = ForwardDynamics(thetaVec, dthetaVec, tauVec, g,...
%    Ftip, Mlist, Glist, Slist);
    [thetaVec, dthetaVec] = RungeKuttaSlover(thetaVec, dthetaVec, dt,...
                        tauVec, g, Ftip, Mlist, Glist, Slist);

    Ftip = zeros(6,1);
%    dthetaVec = dthetaVec + dt * ddthetaVec;
%    thetaVec = thetaVec + dt * dthetaVec;
    cla(ax);
    plotrobot(model,thetaVec,ax);
    title('Time: ' + string(T) + ' s');
end



