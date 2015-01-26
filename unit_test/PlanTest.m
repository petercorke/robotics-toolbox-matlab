function tests = PlanTest
  tests = functiontests(localfunctions);
end

function astar_test(testCase)
    load map1           % load map
    goal = [50;30];
    start = [20;10];
    as = Astar(map);    % create Navigation object
    as.plan(goal);      % setup costmap for specified goal
    as.path(start);     % plan solution path star-goal, animate
    P = as.path(start); % plan solution path star-goal, return path
end

function astarPO_test(testCase)
    load map1          % load map
    goal = [50;30];
    start = [20;10];
    as = Astar(map); % create Navigation object
    as.plan(goal,2);   % setup costmap for specified goal
    as.path(start);    % plan solution path star-goal, animate
    P = as.path(start); % plan solution path star-goal, return path
end

function astarMOO_test(testCase)
    goal = [100;100];
    start = [1;1];
    as = Astar(0);   % create Navigation object with random occupancy grid
    as.addCost(1,L);    % add 1st add'l cost layer L
    as.plan(goal,3);    % setup costmap for specified goal
    as.path(start);     % plan solution path start-goal, animate
    P = as.path(start);    % plan solution path start-goal, return path
end

function bug2_test(testCase)

    goal = [50,30];
    start = [20, 10];
    load map1

    b = Bug2(map);
    s = b.char();
    b.goal = goal;

    b.path(start);
    p = b.path(start);
    b.plot(p);
end

function dstar_test(testCase)

    % load the world
    load map1
    goal = [50,30];
    start = [20, 10]';

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
end

function dxform_test(testCase)
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
end

function prm_test(testCase)
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
end

function rrt_test(testCase)

    goal = [0,0]; 
    randinit

    veh = Vehicle([], 'stlim', 1.2);
    rrt = RRT([], veh, 'goal', goal, 'range', 5);
    s = rrt.char();

    rrt.plan();

    p = rrt.path([0 0 0], [0 2 0]);
end
