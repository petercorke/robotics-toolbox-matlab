clear classes

fprintf('********************* DR test\n');
V = diag([0.005, 0.5*pi/180].^2);
P0 = diag([0.005, 0.005, 0.001].^2);

randinit
v = Vehicle(V);
v.add_driver( RandomPath(10) );

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

f2
ekf.plot_P()

pause
fprintf('********************* mapnav test\n');
W = diag([0.1, 1*pi/180].^2);
P0 = diag([0.005, 0.005, 0.001].^2);
V = diag([0.005, 0.5*pi/180].^2);

randinit
map = Map(20, 10);
veh = Vehicle(V);
veh.add_driver( RandomPath(10) );
sensor = RangeBearingSensor(veh, map, W);
sensor.interval = 5;
ekf = EKF(veh, W, P0, sensor, W);

randinit
ekf.run(1000);



f1
clf
map.visualize()
veh.plot_xy('b');
ekf.plot_xy('r');
ekf.plot_ellipse([], 'k')
grid on
xyzlabel

f2
clf
ekf.plot_P()

pause
fprintf('********************* mapping test\n');
W = diag([0.1, 1*pi/180].^2);
P0 = diag([0.005, 0.005, 0.001].^2);
V = diag([0.005, 0.5*pi/180].^2);

randinit
map = Map(20, 10);
veh = Vehicle(V);
veh.add_driver( RandomPath(10) );
sensor = RangeBearingSensor(veh, map, W);
sensor.interval = 5;
ekf = EKF(veh, [], P0, sensor, W, []);
ekf.verbose = true;

randinit
ekf.run(1000);

f1
clf
map.visualize()
veh.plot_xy('b');
ekf.plot_map('g');
grid on
xyzlabel

pause
fprintf('********************* slam test\n');
W = diag([0.1, 1*pi/180].^2);
P0 = diag([0.005, 0.005, 0.001].^2);
V = diag([0.005, 0.5*pi/180].^2);

randinit
map = Map(20, 10);
veh = Vehicle(V);
veh.add_driver( RandomPath(10) );
sensor = RangeBearingSensor(veh, map, W);
sensor.interval = 1;
ekf = EKF(veh, V, P0, sensor, W, []);
ekf.verbose = true;

ekf.run(1000);

f1
clf
map.visualize()
veh.plot_xy('b');
ekf.plot_xy('r');
ekf.plot_ellipse([], 'k')
grid on
xyzlabel

f2
clf
ekf.plot_P()

f3
map.visualize();
ekf.plot_map(5,'g');
