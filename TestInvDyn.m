clear; clc;
thetaVec = [0.1; 0.1; 0.1];
dthetaVec = [0.1; 0.2; 0.3];
ddthetaVec = [2; 1.5; 1];
g = [0; 0; -9.8];
Ftip = [1; 1; 1; 1; 1; 1];
M01 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.089159]; [0, 0, 0, 1]];
M12 = [[0, 0, 1, 0.28]; [0, 1, 0, 0.13585]; [-1, 0 ,0, 0]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0]; [0, 1, 0, -0.1197]; [0, 0, 1, 0.395]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0]; [0, 1, 0, 0]; [0, 0, 1, 0.14225]; [0, 0, 0, 1]];
G1 = diag([3.7, 3.7, 3.7, 0.010267, 0.010267, 0.00666]);
G2 = diag([8.393, 8.393, 8.393, 0.22689, 0.22689, 0.0151074]);
G3 = diag([2.275, 2.275, 2.275, 0.0494433, 0.0494433, 0.004095]);
Glist = cat(3, G1, G2, G3);
Mlist = cat(3, M01, M12, M23, M34); 
Slist = [[0; 1;     0; 0; 0; 1      ], ...
       [ -0.089; 0;     0; 0; 1; 0], ...
       [ -0.089; 0; 0.425; 0; 1; 0]];
tauVec = InverseDynamics(thetaVec, dthetaVec, ddthetaVec, g,...
                    Ftip, Mlist, Glist, Slist)