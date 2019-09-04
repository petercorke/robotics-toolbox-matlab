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
    
    tc.assumeTrue(ispc || ismac);  % FILTER video generation for Travis
    fname = fullfile(tempdir, 'bug.mp4');
    p = nav.query(tc.TestData.start, tc.TestData.goal, 'animate', 'movie', fname);
    tc.verifyTrue(exist(fname, 'file') == 2);
    delete(fname);
    
    tc.verifyError( @() nav.plan(), 'RTB:Bug2:badcall');
    
    map = zeros(10,10);
    map(3:7,3:7) = 1;
    map(4:6,4:6) = 0;
    nav = Bug2(map);
    tc.verifyError( @() nav.query([5 5], [2 2]), 'RTB:bug2:noplan');
    
    
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

function distancexform_test(tc)
    map = zeros(10,10);
    map(4:6,4:6) =1;
    %args= {'verbose'}
    args = {};
    
    dx1 = distancexform(map, [5 8], 'fast', 'noipt', args{:});
    tc.verifyClass(dx1, 'double');
    tc.verifyEqual(dx1(8,5), 0);
    tc.verifyTrue(all(all(isnan(dx1(map==1)))));
    i=sub2ind(size(dx1), 8, 5);
    tc.verifyTrue(all(dx1(i+[-11 -10 -9 -1 1 11 10 9])) > 0);
    
    tc.verifySize(dx1, size(map));
    dx2 = distancexform(map, [5 8], 'nofast', 'ipt', args{:});
    tc.verifyClass(dx1, 'double');
    tc.verifySize(dx1, size(map));
    dx3 = distancexform(map, [5 8], 'nofast', 'noipt', args{:});
    tc.verifyClass(dx1, 'double');
    tc.verifySize(dx1, size(map));
    
    tc.verifyEqual(dx1, dx2, 'absTol', 1e-6, 'MEX ~= bwdist');
    tc.verifyEqual(dx1, dx3, 'MEX ~= MATLAB');
    
    dx1 = distancexform(map, [5 8], 'cityblock', 'fast', 'noipt', args{:});
    tc.verifyClass(dx1, 'double');
    tc.verifyTrue(all(all(isnan(dx1(map==1)))));
    i=sub2ind(size(dx1), 8, 5);
    tc.verifyTrue(all(dx1(i+[-11 -10 -9 -1 1 11 10 9])) > 0);
    tc.verifySize(dx1, size(map));
    dx2 = distancexform(map, [5 8], 'cityblock', 'nofast', 'ipt', args{:});
    tc.verifyClass(dx1, 'double');
    tc.verifySize(dx1, size(map));
    dx3 = distancexform(map, [5 8], 'cityblock', 'nofast', 'noipt', args{:});
    tc.verifyClass(dx1, 'double');
    tc.verifySize(dx1, size(map));
    
    tc.verifyEqual(dx1, dx2, 'absTol', 1e-6, 'MEX ~= bwdist');
    tc.verifyEqual(dx1, dx3, 'MEX ~= MATLAB');
    
    dx1 = distancexform(map, [5 8], 'cityblock', 'animate');
    dx1 = distancexform(map, [5 8], 'cityblock', 'animate', 'delay', 0.1);
    
    tc.assumeTrue(ispc || ismac);  % FILTER video generation for Travis
    fname = fullfile(tempdir, 'bug.mp4');
    dx1 = distancexform(map, [5 8], 'cityblock', 'animate', 'movie', fname);
    tc.verifyTrue(exist(fname, 'file') == 2);
    delete(fname)
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
