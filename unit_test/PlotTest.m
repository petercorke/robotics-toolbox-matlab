%% This is for testing the Plotting functions in the robotics Toolbox
function tests = TransformationsTest
  tests = functiontests(localfunctions);
end

%    tranimate                  - animate a coordinate frame
function tranimate_test(testCase)
    Rt1 = [1     0     0     0
           0     1     0     0
           0     0     1     0
           0     0     0     1];
    Rt2 = [1.0000         0         0         0
                0    0.8253   -0.5646         0
                0    0.5646    0.8253         0
                0         0         0    1.0000];
    tranimate(Rt1,Rt2);
end

%    trplot                     - plot HT as a coordinate frame
function trplot_test(testCase)
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
end

 function trplot2_test(testCase)
    Rt1 = [
    0.9553   -0.2955    1.0000
    0.2955    0.9553    2.0000
         0         0    1.0000];
    trplot2(Rt1);
    clf
    h = trplot2(Rt1);
    trplot2(h, Rt1);
    trplot2(Rt1, 'color', 'r');
    trplot2(Rt1, 'color', [1 0 1]);
    trplot2(Rt1, 'noaxes');
    trplot2(Rt1, 'frame', 'bob');
    trplot2(Rt1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
    trplot2(Rt1, 'view', [10 20]);
    trplot2(Rt1, 'arrow')
end

%    plot2                      - plot trajectory
function plot2_test(testCase)
        th = [0:0.1:pi];
        sth = sin(th);
        p = [th' sth'];
        plot2(p);

        p = [th' sth' sth'];
        plot2(p);
end
        

%    plot_box                   - draw a box
function plot_box_test(testCase)
        plot_box(1,1,5,5,'b');
        plot_box(2,2,4,4,'g');
end

%    plot_circle                - draw a circle
function plot_circle_test(testCase)
    plot_circle([1 2],2,'g');
    plot_circle([1 2],2,'fillcolor', 'g');
    plot_circle([1 2],2,'fillcolor', 'g', 'alpha', 0.5);
    plot_circle([1 2],2,'edgecolor', 'b');
    plot_circle([1 2],2,'fillcolor', 'g', 'edgecolor', 'b');
end


%    plot_ellipse               - draw an ellipse
function plot_ellipse_test(testCase)
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
function plot_homline_test(testCase)
    plot_homline([1 2 3]','y');
end

%    plot_point                 - plot points
function plot_point_test(testCase)
    plot_point([1; 2; 3]);
end

%    plot_poly                  - plot polygon
function plot_poly_test(testCase)
    p = [1 2 1 2;1 1 2 2];     
    plot_poly(p,'g');
end

%    plot_sphere                - draw a sphere
function plot_sphere_test(testCase)
     plot_sphere([1 2 3],5,'r');
end

%    qplot                      - plot joint angle trajectories
function qplot_test(testCase)
    t = [0:0.5:2]
    mdl_puma560;
    T1 = transl(0.4,0.2,0)*trotx(pi);
    T2 = transl(0.4,-0.2,0)*trotx(pi/2);
    q = p560.jtraj(T1,T2,t);
    qplot(t,q);
end
