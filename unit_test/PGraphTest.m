function test_suite = RobotToolboxQuaternion_test
  initTestSuite;

function PGraph2_test
 
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

    assertEqual(g.n, 5);
    assertEqual(g.ne, 6);
    assertEqual(g.nc, 1);
    assertEqual(g.neighbours(2), [3 4 1]);
    z = g.coord(1);
    assertElementsAlmostEqual( g.distance(1,2), 0.6878, 'absolute',1e-4)
    assertElementsAlmostEqual( g.distances([0 0]), ...
        [0.6137    0.6398    0.9222    1.2183    1.3593], 'absolute',1e-4)

    D = g.degree();
    I = g.incidence();
    A = g.adjacency();
    L = g.laplacian();

    assertEqual(D, diag([3 3 2 3 1]));
    assertEqual(sum(I), [2 2 2 2 2 2]);
    assertEqual(sum(I'), [3 3 2 3 1]);
    assertEqual(max(max(A-A')), 0);
    
    assertEqual( g.edgedir(1,3), 1);
    assertEqual( g.edgedir(3,1), -1);
    assertEqual( g.edgedir(2,5), 0);
    assertEqual(g.neighbours_d(2), [-1 3 4]);


    s = g.char();

    g.setdata(1, [1 2 3]);
    d = g.data(1);
    assertEqual(d, [1 2 3]);

    % add second component

    g.add_node( rand(2,1));
    g.add_node( rand(2,1), 6, 1);
    g.add_node( rand(2,1), 7);

    assertEqual(g.nc, 2);

    clf
    g.plot('labels')
    g.plot('edgelabels')
    g.plot('componentcolor')

    [path,cost]=g.Astar(3,5);
    g.highlight_path(path);
    assertEqual(path, [3 2 4 5]);
    assertElementsAlmostEqual( cost, 2.1536, 'absolute',1e-4)

    g.goal(5);
    path = g.path(3);
    assertEqual(path, [3 2 4 5]);

    assertEqual(g.closest([0 0]), 4);

    g.setcost(1, 99);
    assertEqual( g.cost(1), 99);
    
    g.Astar(3, 5);
    assertEqual(path, [3 2 4 5]);


function PGraph3_test
 
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
