%clear classes
randinit
clf
g = PGraph();
g

v = g.add_node( rand(2,1) );
v
v = g.add_node( rand(2,1) );
v = g.add_node( rand(2,1) );
v = g.add_node( rand(2,1) );
v = g.add_node( rand(2,1) );
v
g
g.plot()
pause

% add nodes 1-4
g.add_edge(1,2);
g.add_edge(1,3);
g.add_edge(1,4);
g.add_edge(2,3);
g.add_edge(2,4);
g.add_edge(4,5);

g
g.plot('labels')

pause

g.neighbours(1)
g.neighbours(4)

g.neighbours2(1)
g.neighbours2(4)

pause

% add nodes 5-8, all connected to node 5
v1 = g.add_node( rand(2,1) );
v = g.add_node( rand(2,1), v1 );
v = g.add_node( rand(2,1), v1 );
v = g.add_node( rand(2,1), v1 );
g
g.plot('labels')
pause

g.add_edge(4,7)
g
g.plot()
pause

% test path finding
g.goal(7)
g.path(1)

