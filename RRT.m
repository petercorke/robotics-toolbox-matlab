%RRT Class for rapidly-exploring random tree navigation
%
% A concrete class that implements the RRT navigation algorithm.
% This class subclasses the Navigation class.
%
% Usage for subclass:
%
%   rrt = RRT(occgrid, options)  create an instance object
%
%   rrt                     show summary statistics about the object
%   rrt.visualize()         display the occupancy grid
%
%   rrt.plan(goal)          plan a path to coordinate goal
%   rrt.path(start)         display a path from start to goal
%   p = rrt.path(start)     return a path from start to goal
%
% Options::
% 'npoints',N    Number of nodes in the tree
% 'time',T       Period to simulate dynamic model toward random point
% 'xrange',X     Workspace span in x-direction [xmin xmax]
% 'yrange',Y     Workspace span in y-direction [ymin ymax]
% 'goal',P       Goal position (1x2) or pose (1x3) in workspace
%
% Notes::
% - The bicycle model is hardwired into the class (should be a parameter)
% - Default workspace is between -5 and +5 in the x- and y-directions
%

% Peter Corke 8/2009.

%TODO
%   more info to the display method
%   distance metric choice or weightings
%   pass time and model options to the simulation

classdef RRT < Navigation

    properties
        npoints         % number of points to find
        graph           % graph Object representing random nodes

        vgoal
        vstart
        localGoal
        localPath
        gpath            % path through the graph

        sim_time        % path simulation time
        kine_model      % simulink model for kinematics

        xrange
        yrange
        
        plant
    end

    methods

        % constructor
        function rrt = RRT(varargin)

            % invoke the superclass constructor
            rrt = rrt@Navigation(varargin{:});

            rrt.graph = PGraph(3, 'distance', 'SE2');  % graph of points in SE(2)

            % TODO: need a means to set these
            %  {'npoints', []
            
            opt.npoints = 500;
            opt.time = 0.5;
            opt.yrange = [-5,5];
            opt.xrange = [-5,5];
            opt.goal = [0, 0];
            
            [opt,args] = tb_optparse(opt, varargin);
            rrt.npoints = opt.npoints;
            rrt.sim_time = opt.time;
            rrt.xrange = opt.xrange;
            rrt.yrange = opt.yrange;
            rrt.goal = opt.goal;
        end

        function plan(rrt)


            % build a graph over the free space
            rrt.message('create the graph');
            rrt.graph.clear();
            rrt.create_graph();
            disp('graph create done');

            return
            disp('plan still going');
            rrt.vgoal = rrt.graph.closest(rrt.goal);

            % find a path through the graph
            rrt.message('planning path through graph');
            rrt.graph.goal(rrt.vgoal);   % set the goal

        end
        

    function p = path(rrt, xstart, xgoal)

        g = rrt.graph;
        vstart = g.closest(xstart)

        vgoal = g.closest(xgoal)
        g.goal(vstart);
        p = g.path(vgoal)

        pp = g.coord(p);

        figure
        clf
        plot2(pp', '-o');
        grid
        xyzlabel

        path = [];
        for k= p(end-1:-1:1)
            b = g.userdata{k};
            path = [path b.path(:,1:b.k)];
        end

        clf
        plot2(path');
            xlabel('x');
            ylabel('y');
            zlabel('\theta');
            grid
        p = path;

    end
    
    function next(rrt)
    end
    
        function create_graph(rrt)

            if rrt.verbose
                clf
                %idisp(1-rrt.occgrid, 'ynormal', 'nogui');
                hold on
            end

            % add the goal point as the first node
            rrt.graph.add_node([rrt.goal(:); 0]);

            j = 0;
            h = waitbar(0, 'RRT planning...');
            
            clf; hold on
            while j < rrt.npoints

                waitbar(j / rrt.npoints);
                
                % Step 3
                % find random state x,y

                % pick a point not in obstacle
                while true
                    xy = rrt.randxy();
                    ixy = round(xy);
                    if isempty(rrt.occgrid)
                        break;
                    end
                    try
                        if rrt.occgrid(ixy(2),ixy(1)) == 0
                            break;
                        end
                    catch
                        % index error, point must be off the map
                        continue;
                    end
                end
                theta = rand*2*pi;
                xrand = [xy, theta]';
                plot(xy(1), xy(2), 'o')

                % Step 4
                % find the existing node closest in state space

                vnear = rrt.graph.closest(xrand);
                xnear = rrt.graph.coord(vnear);

                % Step 5
                % figure how to drive the robot from xnear to xrand
                
                ntrials = 50;
                
                best = bestpath(xnear, xrand, ntrials);
                
                xnew = best.path(:,best.k);
                plot(xnew(1), xnew(2), '+');
                drawnow

%                 % ensure that the path is collision free
%                 if ~rrt.clearpath(y(:,1:2))
%                     disp('path collision');
%                     continue;
%                 end
% 
%                 xnew = y(end,:)';
%                 if rrt.verbose
%                     plot2(y)
%                     plot2(xnew', 'go');
%                     drawnow
%                 end

                % Step 7,8
                % add xnew to the graph, with an edge to xnear
                v = rrt.graph.add_node(xnew, vnear);
                rrt.graph.userdata{v} = best;
                j = j+1;

            end
            close(h)
        end % plan

        function xy = randxy(rrt)
            xy = rand(1,2) .* [rrt.xrange(2)-rrt.xrange(1) rrt.yrange(2)-rrt.yrange(1)] + [rrt.xrange(1) rrt.yrange(1)];
        end

        function c = clearpath(rrt, xy)
            if isempty(rrt.occgrid)
                c = true;
                return;
            end

            xy = round(xy);
            try
                % test that all points along the path do lie within an obstacle
                for pp=xy'
                    if rrt.occgrid(pp(2), pp(1)) > 0
                        c = false;
                        return;
                    end
                end
                c = true;
            catch
                % come here if we index out of bounds
                c = false;
                return;
            end
        end

        function s = char(rrt)
            s = '';
            s = strvcat(s, sprintf('RRT: %dx%d', size(rrt.occgrid)));
            s = strvcat(s, sprintf('  graph size: %d', rrt.npoints));
            s = strvcat(s, char(rrt.graph) );
        end


        function visualize(rrt, varargin)
            %visualize@Navigation(rrt);
            clf
            rrt.graph.plot('noedges', 'MarkerSize', 6, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g');
            hold on
            for i=2:rrt.graph.n
                b = rrt.graph.userdata{i};
                plot2(b.path(:,1:b.k)')
            end
            xlabel('x');
            ylabel('y');
            zlabel('\theta');
            grid
            hold off
        end

    end % method
end % classdef

function x = pathsim(x0, u)

    if nargin < 2
        vel = 1
    end

    tmax = 1;
    L = 0.5;

    f = @(t, x) bicycle(t, x, u, L);

    [t,x] = ode45(f, [0 tmax], x0(:)');
    if nargout < 1
        plot2(x);
    end

end

function xdot = bicycle(t, x, u, L)


    xb = x(1); yb = x(2); thb = x(3);
    v = u(1); gamma = u(2);

    xdot = v * [ cos(thb)
                 sin(thb)
                 tan(gamma) / L ];
end

function best = bestpath(x0, xg, N)
if nargin < 2
    N = 50;
end

x0 = x0(:); xg = xg(:);

steermax = 1.2; % max steer angle in rads
speed = 1;

best.d = Inf;
for i=1:N
    % for multiple trials choose random direction of motion and random
    % steer angle
    if rand > 0.5
        vel = speed;
    else
        vel = -speed;
    end
    steer = (2*rand - 1) * steermax;
    
    % simulate motion for this speed and steer angle
    x = pathsim(x0, [vel steer])';
    
    % find the point on path closest to xg
    d = colnorm( [bsxfun(@minus, x(1:2,:), xg(1:2)); angdiff(x(3,:), xg(3))] );
    
                        %d = sum( (xv(1:2)-xrand(1:2)).^2 ) + angdiff(xv(3)-xrand(3))^2;

    
    % determine closest point to our goal
    [dmin,k] = min(d);
    
    % is it best so far?
    if dmin < best.d
        best.d = dmin;
        best.path = x;
        best.steer = steer;
        best.vel = vel;
        best.k = k;
    end
end
end
