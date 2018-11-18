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

function teardownOnce(tc)
    close all
end

function bug2_test(tc)

    nav = Bug2(tc.TestData.map);
    tc.verifyInstanceOf(nav, 'Bug2');
    s = nav.char();
    verifyTrue(tc, ischar(s) );
    
    nav.plot();

    p = nav.query(tc.TestData.start, tc.TestData.goal);
    
    tc.verifyTrue(size(p,2) == 2, 'plan must have 2 columns');
    tc.verifyEqual(p(1,:), tc.TestData.start, 'start point must be on path');
    tc.verifyEqual(p(end,:), tc.TestData.goal, 'goal point must be on path');
    tc.verifyFalse( any( nav.isoccupied(p') ), 'path must have no occupied cells');
    
    nav.plot()
    nav.plot(p);
end


function dxform_test(tc)

    nav = DXform(tc.TestData.map);
    tc.verifyInstanceOf(nav, 'DXform');

    s = nav.char();
    verifyTrue(tc, ischar(s) );
    
    nav.plot();
    nav.plan(tc.TestData.goal);
    nav.plot();
    nav.plan(tc.TestData.goal, 'animate');
    
    p = nav.query(tc.TestData.start);
    
    tc.verifyTrue(size(p,2) == 2, 'plan must have 2 columns');
    tc.verifyEqual(p(1,:), tc.TestData.start, 'start point must be on path');
    tc.verifyEqual(p(end,:), tc.TestData.goal, 'goal point must be on path');
    tc.verifyFalse( any( nav.isoccupied(p') ), 'path must have no occupied cells');
    
    nav.query(tc.TestData.start, 'animate');
    
    nav.plot();
    nav.plot(p);
    
    nav.plot3d();
    nav.plot3d(p);
end

function dstar_test(tc)

    % create a planner
    nav = Dstar(tc.TestData.map, 'quiet');
    tc.verifyInstanceOf(nav, 'Dstar');
    
    s = nav.char();
    verifyTrue(tc, ischar(s) );
    
    nav.plot();
    
    % plan path to goal
    nav.plan(tc.TestData.goal);
    nav.plan(tc.TestData.goal, 'animate');

    % execute it
    p = nav.query(tc.TestData.start);

        tc.verifyTrue(size(p,2) == 2, 'plan must have 2 columns');
    tc.verifyEqual(p(1,:), tc.TestData.start, 'start point must be on path');
    tc.verifyEqual(p(end,:), tc.TestData.goal, 'goal point must be on path');
    tc.verifyFalse( any( nav.isoccupied(p') ), 'path must have no occupied cells');

    nav.query(tc.TestData.start, 'animate');
    
    nav.plot();
    nav.plot(p);
    
    % add a swamp
    for r=78:85
        for c=12:45
            nav.modify_cost([c;r], 2);
        end
    end
    
    % replan
    nav.plan();

    % show new path
    nav.query(tc.TestData.start);

    p = nav.query(tc.TestData.start);
    tc.verifyTrue(size(p,2) == 2, 'plan must have 2 columns');
    tc.verifyEqual(p(1,:), tc.TestData.start, 'start point must be on path');
    tc.verifyEqual(p(end,:), tc.TestData.goal, 'goal point must be on path');
    tc.verifyFalse( any( nav.isoccupied(p') ), 'path must have no occupied cells');
    
    nav.plot(p);
    
    nav.modify_cost([12 45; 78 85], 2);
    nav.modify_cost([12 13 14; 78 79 80], [2 3 4]);
end

function prm_test(tc)

    randinit
    nav = PRM(tc.TestData.map);
        tc.verifyInstanceOf(nav, 'PRM');

    s = nav.char();
    verifyTrue(tc, ischar(s) );

    nav.plot();

    nav.plan();
    nav.plot();
    
    p = nav.query(tc.TestData.start, tc.TestData.goal);
    
    tc.verifyTrue(size(p,2) == 2, 'plan must have 2 columns');
    tc.verifyEqual(p(1,:), tc.TestData.start, 'start point must be on path');
    tc.verifyEqual(p(end,:), tc.TestData.goal, 'goal point must be on path');
    tc.verifyFalse( any( nav.isoccupied(p') ), 'path must have no occupied cells');
    
    nav.query(tc.TestData.start, tc.TestData.goal);
    nav.plot(p);
end

function lattice_test(tc)
    
    nav = Lattice();
    tc.verifyInstanceOf(nav, 'Lattice');
    
    
    nav.plan('iterations', 8);
    nav.plot()
    
    start = [1 2 pi/2]; goal = [2 -2 0];
    p = nav.query( start, goal );
    verifyEqual(tc, size(p,1), 7);
    tc.verifyTrue(size(p,2) == 3, 'plan must have 3 columns');
    tc.verifyEqual(p(1,:), start, 'AbsTol', 1e-10, 'start point must be on path');
    tc.verifyEqual(p(end,:), goal, 'AbsTol', 1e-10, 'goal point must be on path');
    
    nav.plot

    nav.plan('cost', [1 10 10])
    p = nav.query( start, goal );
    verifyEqual(tc, size(p,1), 9);
    tc.verifyTrue(size(p,2) == 3, 'plan must have 3 columns');
    tc.verifyEqual(p(1,:), start, 'AbsTol', 1e-10, 'start point must be on path');
    tc.verifyEqual(p(end,:), goal, 'AbsTol', 1e-10, 'goal point must be on path');
    
    load road
    nav = Lattice(road, 'grid', 5, 'root', [50 50 0])
    nav.plan();
    start = [30 45 0]; goal = [50 20 0];
    p = nav.query(start, goal);
    tc.verifyTrue(size(p,2) == 3, 'plan must have 3 columns');
    tc.verifyEqual(p(1,:), start, 'AbsTol', 1e-10, 'start point must be on path');
    tc.verifyEqual(p(end,:), goal, 'AbsTol', 1e-10, 'goal point must be on path');
    tc.verifyFalse( any( nav.isoccupied(p(:,1:2)') ), 'path must have no occupied cells');

end

function rrt_test(tc)
    randinit

    car = Bicycle('steermax', 0.5);
    %nav = RRT(car, 'goal', goal, 'range', 5);
    nav = RRT(car, 'npoints', 400);
    tc.verifyInstanceOf(nav, 'RRT');

    s = nav.char();

    nav.plan();
    nav.plot();
    
    start = [0 0 0]; goal = [0 2 0];
    p = nav.query(start, goal);
    
    tc.verifyTrue(size(p,2) == 3, 'plan must have 3 columns');
    tc.verifyFalse( any( nav.isoccupied(p(:,1:2)') ), 'path must have no occupied cells');
    
    % bigger example with obstacle
    load road
    
    randinit
    nav = RRT(car, road, 'npoints', 1000, 'root', [50 22 0], 'simtime', 4);
    nav.plan();
    p = nav.query([40 45 0], [50 22 0]);
    tc.verifyTrue(size(p,2) == 3, 'plan must have 3 columns');
    tc.verifyFalse( any( nav.isoccupied(p(:,1:2)') ), 'path must have no occupied cells');
    
end
