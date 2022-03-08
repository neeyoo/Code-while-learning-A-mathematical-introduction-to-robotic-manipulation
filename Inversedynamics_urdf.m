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


%%
thetaVec = [0; -pi/2; 0; -pi/2; 0; 0];
dthetaVec = [0; 0; 0; 0; 0; 0];
ddthetaVec = [0; 0; 0; 0; 0; 0];
g = [0; 0; -9.8];
Ftip = [0; 0; 0; 0; 0; 0];
plotrobot(model,thetaVec);
tauVec = InverseDynamics(thetaVec, dthetaVec, ddthetaVec, g,...
                    Ftip, Mlist, Glist, Slist)