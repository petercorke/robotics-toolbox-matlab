function nav_bug2

goal = [50,30];
start = [20, 10];
load map1

b = bug2(map);
b.goal = goal;

b.path(start);
p = b.path(start);
b.visualize(p);
