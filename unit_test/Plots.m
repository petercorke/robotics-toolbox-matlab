%% This is for testing the Plotting functions in the robotics Toolbox
function test_suite = TestRobotToolboxPlots
  initTestSuite;
%    tranimate                  - animate a coordinate frame
function Test_tranimate
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
function Test_trplot
Rt1 = [1.0000         0         0         0
            0    0.8253   -0.5646         0
            0    0.5646    0.8253         0
            0         0         0    1.0000];
trplot(Rt1);
%test plot of mastraj
function Test_mastraj
via = [4 1; 4 4; 5 2; 2 5];
mstraj(via, [2 1],[],[4 1],0.05,0)
%% Quaternion 
% Test plof function of Quaternion
function Test_Quaternion
    R = [1.0000         0         0         
         0    0.5403   -0.8415         
         0    0.8415    0.5403];         
    q = Quaternion(R).double;
    plot(q);
%% SerialLink
%    plot                       - plot/animate robot
function Test_SerialLink_plot
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    R1.plot([1 1]);
%    teach                      - drive a graphical  robot
function Test_teach
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    R1.teach;
%% Graphics
%    plot2                      - plot trajectory
function Test_plot2
    rrt = RRT();
    rrt.plan();
    p = rrt.path([0 0 0],[0 2 0]);
    plot2(p');
    
%    plot_box                   - draw a box
function Test_plot_box
    plot_box(1,1,5,5,'b');
    plot_box(2,2,4,4,'g');
%    plot_circle                - draw a circle
function Test_plot_circle
    plot_circle([1 2],2,'g');
%    plot_ellipse               - draw an ellipse
function Test_plot_ellipse
    plot_ellipse([1 0; 0 4],[2 3],'r');
%    plot_homline               - plot homogeneous line
function Test_plot_homline
    plot_homline([1 2 3]','y');
%    plot_point                 - plot points
function Test_plot_point
    plot_point([1; 2; 3]);
%    plot_poly                  - plot polygon
function Test_plot_poly
    p = [1 2 1 2;1 1 2 2];     
    plot_poly(p,'g');
%    plot_sphere                - draw a sphere
function Test_plot_sphere
     plot_sphere([1 2 3],5,'r');
%    qplot                      - plot joint angle trajectories
function Test_qplot
    t = [0:0.5:2]
    mdl_puma560;
    T1 = transl(0.4,0.2,0)*trotx(pi);
    T2 = transl(0.4,-0.2,0)*trotx(pi/2);
    q = p560.jtraj(T1,T2,t);
    qplot(t,q);