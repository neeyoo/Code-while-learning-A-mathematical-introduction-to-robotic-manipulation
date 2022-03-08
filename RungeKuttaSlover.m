function [theta1, dtheta1] = RungeKuttaSlover(theta0, dtheta0, dt,...
                        tauVec, g, Ftip, Mlist, Glist, Slist)

k1 = dtheta0;
l1 = ForwardDynamics(theta0, dtheta0, tauVec, g,...
    Ftip, Mlist, Glist, Slist);
k2 = dtheta0 + l1 * (dt/2);
l2 = ForwardDynamics(theta0 + k1*(dt/2), dtheta0+l1*(dt/2), tauVec, g,...
    Ftip, Mlist, Glist, Slist);
k3 = dtheta0 + l2*(dt/2);
l3 = ForwardDynamics(theta0 + k2*(dt/2), dtheta0+l2*(dt/2), tauVec, g,...
    Ftip, Mlist, Glist, Slist);
k4 = dtheta0 + l3*dt;
l4 = ForwardDynamics(theta0 + k3*dt, dtheta0+l3*dt, tauVec, g,...
    Ftip, Mlist, Glist, Slist);

theta1 = theta0 + (dt/6) * (k1+2*k2+2*k3+k4);
dtheta1 = dtheta0 + (dt/6) * (l1+2*l2+2*l3+l4);


end