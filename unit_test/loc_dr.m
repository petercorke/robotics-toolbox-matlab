clear classes

V = diag([0.005, 0.5*pi/180].^2);
P0 = diag([0.005, 0.005, 0.001].^2);

randinit
v = Vehicle(V);
r = RandomPath(v, 10);

ekf = EKF(v, V, P0);

randinit
ekf.run(1000);

f1
clf
ekf.plot_xy
hold on 
v.plot_xy('r')
grid on
xyzlabel

ekf.plot_ellipse([], 'g')

p0 = ekf.plot_P;

ekf = EKF(v, 2*V, P0);
randinit
ekf.run(1000);
pb = ekf.plot_P;

ekf = EKF(v, 0.5*V, P0);
randinit
ekf.run(1000);
ps = ekf.plot_P;

f2
clf
plot([p0 pb ps]);
legend('1', '2', '0.5');
grid
