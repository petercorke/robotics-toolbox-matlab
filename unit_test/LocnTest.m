function tests = LocnTest
  tests = functiontests(localfunctions);
end

function Vehicle_test(testCase)
    randinit
    V = diag([0.005, 0.5*pi/180].^2);

    v = Vehicle(V);
    v.add_driver( RandomPath(10) );

    v.run(100);
    v.plot_xy();
    s = v.char();

    J = v.Fx(v.x, [.1 .2]);
    J = v.Fv(v.x, [.1 .2]);
end

function DeadReckoning_test(testCase)
    randinit
    V = diag([0.005, 0.5*pi/180].^2);
    P0 = diag([0.005, 0.005, 0.001].^2);

    v = Vehicle(V);
    v.add_driver( RandomPath(10) );
    s = char(v);

    ekf = EKF(v, V, P0);
    ekf.run(1000);

    clf
    ekf.plot_xy
    hold on 
    v.plot_xy('r')
    grid on
    xyzlabel

    ekf.plot_ellipse([], 'g')
    ekf.plot_P()
end

function MapLocalization_test(testCase)
    randinit
    W = diag([0.1, 1*pi/180].^2);
    P0 = diag([0.005, 0.005, 0.001].^2);
    V = diag([0.005, 0.5*pi/180].^2);

    map = Map(20);
    map = Map(20, 'verbose');
    map = Map(20, 10, 'verbose');
    map = Map(20, 10);
    s = char(map);

    veh = Vehicle(V);
    veh.add_driver( RandomPath(10) );
    sensor = RangeBearingSensor(veh, map, W);
    sensor.interval = 5;
    ekf = EKF(veh, W, P0, sensor, W, map);

    ekf.run(1000);

    clf
    map.plot()
    veh.plot_xy('b');
    ekf.plot_xy('r');
    ekf.plot_ellipse([], 'k')
    grid on
    xyzlabel

    clf
    ekf.plot_P()
end

function Mapping_test(testCase)
    randinit
    W = diag([0.1, 1*pi/180].^2);
    V = diag([0.005, 0.5*pi/180].^2);

    map = Map(20, 10);

    veh = Vehicle(V);
    veh.add_driver( RandomPath(10) );

    sensor = RangeBearingSensor(veh, map, W);
    sensor.interval = 5;

    ekf = EKF(veh, [], [], sensor, W, []);
    ekf.run(1000);
    
    verifyEqual(testCase, numcols(ekf.features), 20);

    clf
    map.plot()
    veh.plot_xy('b');
    ekf.plot_map('g');
    grid on
    xyzlabel
end

function SLAM_test(testCase)
    randinit
    W = diag([0.1, 1*pi/180].^2);
    P0 = diag([0.005, 0.005, 0.001].^2);
    V = diag([0.005, 0.5*pi/180].^2);

    map = Map(20, 10);

    veh = Vehicle(V);
    veh.add_driver( RandomPath(10) );

    sensor = RangeBearingSensor(veh, map, W);
    sensor.interval = 1;

    ekf = EKF(veh, V, P0, sensor, W, []);
    ekf.verbose = false;
    ekf.run(1000);

    verifyEqual(testCase, numcols(ekf.features), 20);

    clf
    map.plot()
    veh.plot_xy('b');
    ekf.plot_xy('r');
    ekf.plot_ellipse([], 'k')
    grid on
    xyzlabel

    clf
    ekf.plot_P()

    clf
    map.plot();
    ekf.plot_map(5,'g');
end

function ParticleFilter_test(testCase)

    randinit
    map = Map(20);

    W = diag([0.1, 1*pi/180].^2);
    v = Vehicle(W);
    v.add_driver( RandomPath(10) );
    V = diag([0.005, 0.5*pi/180].^2);
    sensor = RangeBearingSensor(v, map, V);

    Q = diag([0.1, 0.1, 1*pi/180]).^2;
    L = diag([0.1 0.1]);
    pf = ParticleFilter(v, sensor, Q, L, 1000);
    pf.run(200);

    plot(pf.std)
    xlabel('time step')
    ylabel('standard deviation')
    legend('x', 'y', '\theta')
    grid       

    clf
    pf.plot_pdf();
    clf
    pf.plot_xy();
end

function RRT_test(testCase)

    randinit
    veh = Vehicle([], 'stlim', 1.2);
    rrt = RRT('vehicle', veh);
    rrt.plan();
    rrt.plot();
    rrt.plot('noprogress');
    rrt.plot('samples');

    p = rrt.path([0 0 0], [0 2 0]);
    rrt.path([0 0 0], [0 2 0]);
end
