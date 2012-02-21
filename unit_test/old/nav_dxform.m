function nav_dxform
goal = [50,30];
start = [20, 10];
load map1

dx = DXform(map);
dx

dx.plan(goal);
dx

dx.path(start);

p = dx.path(start);
dx.visualize(p);

