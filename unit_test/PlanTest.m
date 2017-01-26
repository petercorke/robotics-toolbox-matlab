function tests = PlanTest
  tests = functiontests(localfunctions);
end

function setupOnce(tc)
    clc
    load map1          % load map
    tc.TestData.map = map;
    tc.TestData.goal = [50 30];
    tc.TestData.start = [20 10];
end

function navigation_test(tc)
    map = zeros(10,10);
    map(2,3) = 1;
    
    % test isoccupied test
    n = Bug2(map);
    tc.verifyTrue( n.isoccupied([3,2]) );
    tc.verifyFalse( n.isoccupied([3,3]) );

    % test inflation option
    n = Bug2(map, 'inflate', 1);
    tc.verifyTrue( n.isoccupied([3,2]) );
    tc.verifyTrue( n.isoccupied([3,3]) );
    tc.verifyFalse( n.isoccupied([3,4]) );
    
end

function bug2_test(tc)

    b = Bug2(tc.TestData.map);
    s = b.char();
    verifyTrue(tc, ischar(s) );

    p = b.query(tc.TestData.start, tc.TestData.goal);
    verifyTrue(tc, size(p,2) == 2);
    
    b.plot()
    b.plot(p);
end

function dstar_test(tc)

    % create a planner
    ds = Dstar(tc.TestData.map, 'quiet');
    s = ds.char();
    verifyTrue(tc, ischar(s) );
    
    % plan path to goal
    ds.plan(tc.TestData.goal);
    ds.plan(tc.TestData.goal, 'animate');

    % execute it
    p = ds.query(tc.TestData.start);
    verifyTrue(tc, size(p,2) == 2);

    ds.query(tc.TestData.start, 'animate');
    
    ds.plot();
    ds.plot(p);
    
    % add a swamp
    for r=78:85
        for c=12:45
            ds.modify_cost([c;r], 2);
        end
    end
    
    % replan
    ds.plan();

    % show new path
    ds.query(tc.TestData.start)

    p = ds.query(tc.TestData.start);
    ds.plot(p);
    
    ds.modify_cost([12 45; 78 85], 2);
    ds.modify_cost([12 13 14; 78 79 80], [2 3 4]);
end

function dxform_test(tc)

    dx = DXform(tc.TestData.map);
    s = dx.char();
    verifyTrue(tc, ischar(s) );


    dx.plan(tc.TestData.goal);
    dx.plot();
    dx.plan(tc.TestData.goal, 'animate');


    p = dx.query(tc.TestData.start);
    verifyTrue(tc, size(p,2) == 2);
    
    dx.query(tc.TestData.start, 'animate');

    dx.plot();
    dx.plot(p);

    dx.plot3d();
    dx.plot3d(p);
end

function prm_test(tc)

    randinit
    prm = PRM(tc.TestData.map);
    s = prm.char();
    verifyTrue(tc, ischar(s) );


    prm.plan();
    prm.plot();

    prm.query(tc.TestData.start, tc.TestData.goal)
    p = prm.query(tc.TestData.start, tc.TestData.goal);
    prm.plot(p);
end

function lattice_test(tc)
    
    lp = Lattice();
    
    lp.plan('iterations', 8)
    lp.plot()
    
    p = lp.query( [1 2 pi/2], [2 -2 0] );
    verifyEqual(tc, size(p,1), 7);

    lp.plot

    lp.plan('cost', [1 10 10])
    p = lp.query([1 2 pi/2], [2 -2 0]);
    verifyEqual(tc, size(p,1), 9);
    
    load road
    lp = Lattice(road, 'grid', 5, 'root', [50 50 0])
    lp.plan();
    lp.query([30 45 0], [50 20 0])
end



function rrt_test(tc)

    goal = [0,0]; 
    randinit

    car = Bicycle('steermax', 1.2);
    rrt = RRT(car, 'goal', goal, 'range', 5);
    s = rrt.char();

    rrt.plan();

    rrt.plot();
    
    p = rrt.query([0 0 0], [0 2 0]);
    
    % bigger example with obstacle
    load road

     rrt = RRT(car, road, 'npoints', 1000, 'root', [50 22 0], 'simtime', 4);
     rrt.plan();
     
end
