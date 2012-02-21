clear
clf
randinit
map = Map(20);

W = diag([0.1, 1*pi/180].^2);
v = Vehicle(W);
v.add_driver( RandomPath(10) );
V = diag([0.005, 0.5*pi/180].^2);
sensor = RangeBearingSensor(v, map, V);

Q = diag([0.1, 0.1, 1*pi/180]).^2;
L = diag([0.1 0.1])
pf = ParticleFilter(v, sensor, Q, L, 1000);
pf.run(1000);
pause
plot(pf.std)
xlabel('time step')
ylabel('standard deviation')
legend('x', 'y', '\theta')
grid       
