randinit
clf
hold on
g = PGraph(3);

sim_time = 2;

xinit = [0, 0, 0];
plot(xinit(1), xinit(2), 'go');
rho = 1;

g.add_node(xinit);
t =2;

for k=1:500
    % Step 3
    % find random state x,y in [-10, 10], th in [0 2pi]
    x = (rand-0.5)*20;
    y = (rand-0.5)*20;
    theta = rand*2*pi;
    xrand = [x, y, theta]';
    xrand'

    % Step 4
    % find the existing node closest in state space
    dmin = Inf;
    for v=1:g.n
        xv = g.coord(v);
        d = sum( (xv(1:2)-xrand(1:2)).^2 ) + t*angdiff(xv(3)-xrand(3))^2;
        if d < dmin
            xnear = xv;
            vnear = v;
            dmin = d;
        end
    end

    % Step 5
    % figure how to drive the robot from xnear to xrand
    x0 = xnear';
    xg = xrand';
    %set_param('sl_movepoint2/Bicycle', 'x0', sprintf('[%f,%f,%f]', xnear) );
    %set_param('sl_movepoint2/Goal pose', 'Value', sprintf('trans2([%f,%f,0])', xrand(1:2)) );

    r = sim('sl_drivepose', sim_time);
    t = r.find('tout');
    y = r.find('yout');
    plot2(y)
    drawnow
    xnew = y(end,:)';
    plot2(xnew', 'go');

    % ensure that the path is collision free

    % Step 7,8
    % add xnew to the graph, with an edge to xnear
    g.add_node(xnew, vnear);
end

grid on
xyzlabel
zlabel('\theta');
axis([-10 10 -10 10])
iprint('rrt_paths');
view(0, 0);
iprint('rrt_paths_xt');
