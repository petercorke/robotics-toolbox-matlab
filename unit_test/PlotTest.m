%% This is for testing the Plotting functions in the robotics Toolbox
function test_suite = TestRobotToolboxPlots
  initTestSuite;

%    tranimate                  - animate a coordinate frame
function tranimate_test
    Rt1 = [1     0     0     0
           0     1     0     0
           0     0     1     0
           0     0     0     1];
    Rt2 = [1.0000         0         0         0
                0    0.8253   -0.5646         0
                0    0.5646    0.8253         0
                0         0         0    1.0000];
    tranimate(Rt1,Rt2);

%    trplot                     - plot HT as a coordinate frame
function trplot_test
    Rt1 = [1.0000         0         0         0
                0    0.8253   -0.5646         0
                0    0.5646    0.8253         0
                0         0         0    1.0000];
    trplot(Rt1);
    clf
    h = trplot(Rt1);
    trplot(h, Rt1);
    trplot(Rt1, 'color', 'r');
    trplot(Rt1, 'color', [1 0 1]);
    trplot(Rt1, 'noaxes');
    trplot(Rt1, 'frame', 'bob');
    trplot(Rt1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
    trplot(Rt1, 'view', [10 20]);
    trplot(Rt1, 'arrow')
    trplot(Rt1, '3d')
    trplot(Rt1, '3d', 'anaglyph', 'mo')
    trplot(Rt1, '3d', 'dispar', 0.3);

%    plot2                      - plot trajectory
function plot2_test
        th = [0:0.1:pi];
        sth = sin(th);
        p = [th' sth'];
        plot2(p);

        p = [th' sth' sth'];
        plot2(p);
        

%    plot_box                   - draw a box
function plot_box_test
        plot_box(1,1,5,5,'b');
        plot_box(2,2,4,4,'g');

%    plot_circle                - draw a circle
function plot_circle_test
        plot_circle([1 2],2,'g');

%    plot_ellipse               - draw an ellipse
function plot_ellipse_test
        plot_ellipse([1 0; 0 4],[2 3],'r');

%    plot_homline               - plot homogeneous line
function plot_homline_test
        plot_homline([1 2 3]','y');

%    plot_point                 - plot points
function plot_point_test
        plot_point([1; 2; 3]);

%    plot_poly                  - plot polygon
function plot_poly_test
        p = [1 2 1 2;1 1 2 2];     
        plot_poly(p,'g');

%    plot_sphere                - draw a sphere
function plot_sphere_test
         plot_sphere([1 2 3],5,'r');

%    qplot                      - plot joint angle trajectories
function qplot_test
        t = [0:0.5:2]
        mdl_puma560;
        T1 = transl(0.4,0.2,0)*trotx(pi);
        T2 = transl(0.4,-0.2,0)*trotx(pi/2);
        q = p560.jtraj(T1,T2,t);
        qplot(t,q);
