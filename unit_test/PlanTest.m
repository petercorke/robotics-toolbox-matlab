function suite = TestPlanningClasses_test
  initTestSuite;

function bug2_test

    goal = [50,30];
    start = [20, 10];
    load map1

    b = Bug2(map);
    s = b.char();
    b.goal = goal;

    b.path(start);
    p = b.path(start);
    b.plot(p);

function dstar_test

    % load the world
    load map1
    goal = [50,30];
    start = [20, 10];

    % create a planner
    ds = Dstar(map, 'quiet');
    s = ds.char();

    % plan path to goal
    ds.plan(goal);

    % execute it
    ds.path(start);
    p = ds.path(start);

    % add a swamp
    for r=78:85
        for c=12:45
            ds.modify_cost([c,r], 2);
        end
    end

    % replan
    ds.plan();

    % show new path
    ds.path(start)

    p = ds.path(start);
    ds.plot(p);

function dxform_test
    goal = [50,30];
    start = [20, 10];
    load map1

    dx = DXform(map);
    s = dx.char();

    dx.plan(goal);
    dx.plot();
    dx.plan(goal, 0.2);


    dx.path(start);
    p = dx.path(start);

    p = dx.path(start);
    dx.plot(p);

    dx.plot3d();
    dx.plot3d(p);

function prm_test
    load map1
    randinit
    prm = PRM(map);
    s = prm.char();
    goal = [50,30];
    start=[20,10];

    prm.plan();
    prm.plot();

    prm.path(start, goal)
    p = prm.path(start, goal);
    prm.plot(p);

function rrt_test

    goal = [0,0]; 
    randinit

    veh = Vehicle([], 'stlim', 1.2);
    rrt = RRT([], veh, 'goal', goal, 'range', 5);
    s = rrt.char();

    rrt.plan();

    p = rrt.path([0 0 0], [0 2 0]);

