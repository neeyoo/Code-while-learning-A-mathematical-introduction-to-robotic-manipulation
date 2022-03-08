function plotrobot(model, thetaVec, ax1)
axis([-1 1 -1 1 -1 1])
%figure;
hold on;
%% plot the base coordinate
plotcoord([ 0 0 0], [1 0 0], [0 1 0], [0 0 1], ax1);
axis equal
view([1 1 1])

[X,Y,Z] = cylinder(0.015);
Z = Z * 0.1 - 0.05;
%% plot all the joint coordinate
Trans = eye(4);
pprev = zeros(3,1);
for i = 1:length(model.joint)
   Trans = Trans * model.joint{i}.Trans;
   R = Trans(1:3, 1:3);
   p = Trans(1:3, 4);
   plot3(ax1, [pprev(1) p(1)], [pprev(2) p(2)], [pprev(3) p(3)], 'k'); % link the child and paraent frame
   if isfield(model.joint{i}, 'Axis')
        R = R * exposkew( model.joint{i}.Axis', thetaVec(i));
 
       if [0 0 1] * model.joint{i}.Axis' == -1
          rotax = R * [1 0 0; 0 -1 0; 0 0 -1];
       else
           rotax = R * twoVecRot([0 0 1]', model.joint{i}.Axis');
       end
       Xj = p(1) + rotax(1,1) * X + rotax(1, 2) * Y + rotax(1, 3) * Z;
       Yj = p(2) + rotax(2,1) * X + rotax(2, 2) * Y + rotax(2, 3) * Z;
       Zj = p(3) + rotax(3,1) * X + rotax(3, 2) * Y + rotax(3, 3) * Z;
       surf(ax1, Xj,Yj,Zj,'FaceColor','c','EdgeColor','c');       
   end
   plotcoord(p, Trans(1:3, 1), Trans(1:3, 2), Trans(1:3, 3), ax1); %plot joint frame
   Trans(1:3, 1:3) = R;
   pprev = p;
end

hold off;
drawnow;


end