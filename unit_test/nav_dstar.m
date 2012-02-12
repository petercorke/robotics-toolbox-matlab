function nav_dstar

% load the world
load map1
goal = [50,30];
start = [20, 10];

% create a planner
ds = Dstar(map);
ds

% plan path to goal
ds.plan(goal);
ds

% execute it
ds.path(start);

pause(2)

% add a swamp
for r=78:85
    for c=12:45
        ds.modify_cost([c,r], 2);
    end
end

% replan
ds.plan();

% show new path
ds.path(start)

p = ds.path(start);
ds.visualize(p);
