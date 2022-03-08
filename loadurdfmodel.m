function model = loadurdfmodel(file)


s = xml2struct(file);
robot = s.robot;
model.name = robot.Attributes.name;
model.dof = length(robot.transmission);

%% Creat links
model.link = {};
for i = 1:length(robot.link) - 1
    model.link{i}.name = robot.link{i}.Attributes.name;
    model.link{i}.Trans = urdfTrans(robot.link{i}.inertial.origin.Attributes);
    model.link{i}.mass = str2double(robot.link{i}.inertial.mass.Attributes.value);
    ixx = str2double(robot.link{i}.inertial.inertia.Attributes.ixx);
    ixy = str2double(robot.link{i}.inertial.inertia.Attributes.ixy);
    ixz = str2double(robot.link{i}.inertial.inertia.Attributes.ixz);
    iyy = str2double(robot.link{i}.inertial.inertia.Attributes.iyy);
    iyz = str2double(robot.link{i}.inertial.inertia.Attributes.iyz);
    izz = str2double(robot.link{i}.inertial.inertia.Attributes.izz);
    model.link{i}.inertia = [ixx ixy ixz; ixy iyy iyz; ixz iyz izz];   
end

%% Creat joints
model.joint = {};
for i = 1:length(robot.joint)
   model.joint{i}.name = robot.joint{i}.Attributes.name;
   model.joint{i}.type = robot.joint{i}.Attributes.type;
   model.joint{i}.parent = robot.joint{i}.parent.Attributes.link;
   model.joint{i}.child = robot.joint{i}.child.Attributes.link;
   model.joint{i}.Trans = urdfTrans(robot.joint{i}.origin.Attributes);
   if ~strcmp('fixed', model.joint{i}.type)
       model.joint{i}.Axis = str2num(robot.joint{i}.axis.Attributes.xyz);
       model.joint{i}.poslim = [str2double(robot.joint{i}.limit.Attributes.lower) ...
           str2double(robot.joint{i}.limit.Attributes.upper)];
       model.joint{i}.vellim = str2double(robot.joint{i}.limit.Attributes.velocity);
       model.joint{i}.effortlim = str2double(robot.joint{i}.limit.Attributes.effort);
   end
end

end