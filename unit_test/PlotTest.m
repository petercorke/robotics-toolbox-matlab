%% This is for testing the Plotting functions in the robotics Toolbox
function tests = TransformationsTest
  tests = functiontests(localfunctions);
  clc
end

function teardownOnce(tc)
    close all
end

%    tranimate                  - animate a coordinate frame
function tranimate_test(tc)
    X1 = eye(3,3); X2 = rotx(pi/2);
    tranimate(X1, X2);
    tranimate(X1, X2, 'nsteps', 10);
    
    clf
    tranimate(X1, X2, 'axis', [-10 10 -20 20 -30 30]);
    v = axis;
    tc.verifyEqual(v, [-10 10 -20 20 -30 30]);
    
    tranimate(X1, X2, 'noxyz');
    tranimate(X1, X2, 'rgb');
    tranimate(X1, X2, 'retain');
    tranimate(X1, X2, 'fps', 20);
    
    
    X1 = eye(4,4); X2 = transl(1,2,3)*trotx(pi/2);
    tranimate(X1, X2);
    tranimate(X1, X2, 'nsteps', 10);
    
    clf
    tranimate(X1, X2, 'axis', [-10 10 -20 20 -30 30]);
    v = axis;
    tc.verifyEqual(v, [-10 10 -20 20 -30 30]);
    
    tranimate(X1, X2, 'noxyz');
    tranimate(X1, X2, 'rgb');
    tranimate(X1, X2, 'retain');
    tranimate(X1, X2, 'fps', 20);
end

function tranimate2_test(tc)
    X1 = eye(2,2); X2 = rot2(pi/2);
    tranimate2(X1, X2);
    tranimate2(X1, X2, 'nsteps', 10);
    
    clf
    tranimate2(X1, X2, 'axis', [-10 10 -20 20]);
    v = axis;
    tc.verifyEqual(v, [-10 10 -20 20]);
    
    tranimate2(X1, X2, 'noxyz');
    tranimate2(X1, X2, 'rgb');
    tranimate2(X1, X2, 'retain');
    tranimate2(X1, X2, 'fps', 20);
    
    
    X1 = eye(3,3); X2 = transl2(1,2)*trot2(pi/2);
    tranimate2(X1, X2);
    tranimate2(X1, X2, 'nsteps', 10);
    
    clf
    tranimate2(X1, X2, 'axis', [-10 10 -20 20]);
    v = axis;
    tc.verifyEqual(v, [-10 10 -20 20]);
    
    tranimate2(X1, X2, 'noxyz');
    tranimate2(X1, X2, 'rgb');
    tranimate2(X1, X2, 'retain');
    tranimate2(X1, X2, 'fps', 20);
end

%    trplot                     - plot HT as a coordinate frame
function trplot_test(tc)
    %%
    Rt1 = [1.0000         0         0         0
                0    0.8253   -0.5646         0
                0    0.5646    0.8253         0
                0         0         0    1.0000];
    trplot(Rt1);
    clf
    h = trplot(Rt1);
    trplot(Rt1, 'handle', h);
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
end

 function trplot2_test(tc)
     %%
    Rt1 = [
    0.9553   -0.2955    1.0000
    0.2955    0.9553    2.0000
         0         0    1.0000];
    trplot2(Rt1);
    clf
    h = trplot2(Rt1);
    trplot2(Rt1, 'handle', h);
    trplot2(Rt1, 'color', 'r');
    trplot2(Rt1, 'color', [1 0 1]);
    trplot2(Rt1, 'noaxes');
    trplot2(Rt1, 'frame', 'bob');
    trplot2(Rt1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
    trplot2(Rt1, 'view', [10 20]);
    trplot2(Rt1, 'arrow')
end

%    plot2                      - plot trajectory
function plot2_test(tc)
    %%
        th = [0:0.1:pi];
        sth = sin(th);
        p = [th' sth'];
        plot2(p);

        p = [th' sth' sth'];
        plot2(p);
end
        

%    plot_box                   - draw a box
function plot_box_test(tc)
        plot_box(1,1,5,5,'b');
        plot_box(2,2,4,4,'g');
end

%    plot_circle                - draw a circle
function plot_circle_test(tc)
    plot_circle([1 2],2,'g');
    plot_circle([1 2],2,'fillcolor', 'g');
    plot_circle([1 2],2,'fillcolor', 'g', 'alpha', 0.5);
    plot_circle([1 2],2,'edgecolor', 'b');
    plot_circle([1 2],2,'fillcolor', 'g', 'edgecolor', 'b');
end


%    plot_ellipse               - draw an ellipse
function plot_ellipse_test(tc)
    %%
    %2d
    
    C = diag([1,4]);
    plot_ellipse(C,[2 3],'r');
    plot_ellipse(C,[2 3],'fillcolor', 'g');
    plot_ellipse(C,[2 3],'fillcolor', 'g', 'alpha', 0.5);
    plot_ellipse(C,[2 3],'edgecolor', 'g');
    plot_ellipse(C,[2 3],'fillcolor', 'g', 'edgecolor', 'b');
    
    % with 3d centre
    plot_ellipse([1 0; 0 4],[2 3 1],'r');
    
    %3d
    
    C = diag([1,4,2]);
    plot_ellipse(C,[2 3 1],'r');
    plot_ellipse(C,[2 3 1],'fillcolor', 'g');
    plot_ellipse(C,[2 3 1],'fillcolor', 'g', 'alpha', 0.5);
    plot_ellipse(C,[2 3 1],'edgecolor', 'g');
    plot_ellipse(C,[2 3 1],'fillcolor', 'g', 'edgecolor', 'b');
end

%    plot_homline               - plot homogeneous line
function plot_homline_test(tc)
    plot_homline([1 2 3]','y');
end

%    plot_point                 - plot points
function plot_point_test(tc)
    plot_point([1; 2]);
end

%    plot_poly                  - plot polygon
function plot_poly_test(tc)
    p = [1 2 2 1; 1 1 2 2];     
    plot_poly(p,'g');
end

%    plot_sphere                - draw a sphere
function plot_sphere_test(tc)
     plot_sphere([1 2 3],5,'r');
end

%    qplot                      - plot joint angle trajectories
function qplot_test(tc)
    t = [0:0.5:20];
    qz = [0 0 0 0 0 0]; qr = [1 2 3 4 5 6];
    q = jtraj(qz, qr, t);
    clf
    qplot(t,q);
    
    qplot(q);
end
