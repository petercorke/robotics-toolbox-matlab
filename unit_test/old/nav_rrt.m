clear classes

%load map1
%goal = [50,30]; 
%start = [20, 10];

map = zeros(100,100);
goal = [0,0]; 
start = [20, 10];

randinit
rrt = RRT([], goal)
rrt.xrange = [-5 5]
rrt.yrange = [-5 5]
rrt.verbose = true;

rrt.plan

%rrt.visualize
