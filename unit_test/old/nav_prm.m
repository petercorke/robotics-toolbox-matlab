function nav_prm
load map1
randinit
prm = PRM(map)
goal = [50,30]
start=[20,10]

prm.plan();
prm
prm.visualize();

pause(4)

prm.path(start, goal)
p = prm.path(start, goal)
prm.visualize(p);
