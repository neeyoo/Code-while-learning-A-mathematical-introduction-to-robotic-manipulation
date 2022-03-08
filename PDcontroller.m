function tauVec = PDcontroller(Kp, Kd, Xe, Ve, J, M, Gra)

Jinv = inv(J);
tauVec = J'*((Jinv'* M * Jinv) * (Kp * Xe + Kd * Ve)) + Gra;
end