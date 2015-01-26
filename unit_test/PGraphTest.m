function tests = PGraphTest
  tests = functiontests(localfunctions);
end
function PGraph2_test(testCase)
 
    randinit
    g = PGraph(2);

    g.add_node( rand(2,1));
    g.add_node( rand(2,1));
    g.add_node( rand(2,1));
    g.add_node( rand(2,1));
    g.add_node( rand(2,1));

    g.add_edge(1,2);
    g.add_edge(1,3);
    g.add_edge(1,4);
    g.add_edge(2,3);
    g.add_edge(2,4);
    g.add_edge(4,5);

    verifyEqual(testCase, g.n, 5);
    verifyEqual(testCase, g.ne, 6);
    verifyEqual(testCase, g.nc, 1);
    verifyEqual(testCase, g.neighbours(2), [3 4 1]);
    z = g.coord(1);
    verifyEqual(testCase,  g.distance(1,2), 0.6878, 'absTol',1e-4)
    verifyEqual(testCase,  g.distances([0 0]), ...
        [0.6137    0.6398    0.9222    1.2183    1.3593], 'absTol',1e-4)

    D = g.degree();
    I = g.incidence();
    A = g.adjacency();
    L = g.laplacian();

    verifyEqual(testCase, D, diag([3 3 2 3 1]));
    verifyEqual(testCase, sum(I), [2 2 2 2 2 2]);
    verifyEqual(testCase, sum(I'), [3 3 2 3 1]);
    verifyEqual(testCase, max(max(A-A')), 0);
    
    verifyEqual(testCase,  g.edgedir(1,3), 1);
    verifyEqual(testCase,  g.edgedir(3,1), -1);
    verifyEqual(testCase,  g.edgedir(2,5), 0);
    verifyEqual(testCase, g.neighbours_d(2), [-1 3 4]);


    s = g.char();

    g.setdata(1, [1 2 3]);
    d = g.data(1);
    verifyEqual(testCase, d, [1 2 3]);

    % add second component

    g.add_node( rand(2,1));
    g.add_node( rand(2,1), 6, 1);
    g.add_node( rand(2,1), 7);

    verifyEqual(testCase, g.nc, 2);

    clf
    g.plot('labels')
    g.plot('edgelabels')
    g.plot('componentcolor')

    [path,cost]=g.Astar(3,5);
    g.highlight_path(path);
    verifyEqual(testCase, path, [3 2 4 5]);
    verifyEqual(testCase,  cost, 2.1536, 'absTol',1e-4)

    g.goal(5);
    path = g.path(3);
    verifyEqual(testCase, path, [3 2 4 5]);

    verifyEqual(testCase, g.closest([0 0]), 4);

    g.setcost(1, 99);
    verifyEqual(testCase,  g.cost(1), 99);
    
    g.Astar(3, 5);
    verifyEqual(testCase, path, [3 2 4 5]);
end

function PGraph3_test(testCase)
 
    randinit
    g = PGraph(3);

    g.add_node( rand(3,1));
    g.add_node( rand(3,1));
    g.add_node( rand(3,1));
    g.add_node( rand(3,1));
    g.add_node( rand(3,1));

    g.add_edge(1,2);
    g.add_edge(1,3);
    g.add_edge(1,4);
    g.add_edge(2,3);
    g.add_edge(2,4);
    g.add_edge(4,5);

    clf
    g.plot('labels')
    g.plot('edgelabels')
end
