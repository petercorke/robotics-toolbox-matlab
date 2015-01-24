%RRT Class for rapidly-exploring random tree navigation
%
% A concrete subclass of the abstract Navigation class that implements the rapidly
% exploring random tree (RRT) algorithm.  This is a kinodynamic planner
% that takes into account the motion constraints of the vehicle.
%
% Methods::
%
% plan         Compute the tree
% path         Compute a path 
% plot         Display the tree
% display      Display the parameters in human readable form
% char         Convert to string
%
% Example::
%
%        goal = [0,0,0];
%        start = [0,2,0];
%        veh = Vehicle([], 'stlim', 1.2);
%        rrt = RRT([], veh, 'goal', goal, 'range', 5);
%        rrt.plan()             % create navigation tree
%        rrt.path(start, goal)  % animate path from this start location
%
%  Robotics, Vision & Control compatability mode:
%        goal = [0,0,0];
%        start = [0,2,0];
%        rrt = RRT();           % create navigation object
%        rrt.plan()             % create navigation tree
%        rrt.path(start, goal)  % animate path from this start location
%
% References::
% - Randomized kinodynamic planning,
%   S. LaValle and J. Kuffner, 
%   International Journal of Robotics Research vol. 20, pp. 378-400, May 2001.
% - Probabilistic roadmaps for path planning in high dimensional configuration spaces,
%   L. Kavraki, P. Svestka, J. Latombe, and M. Overmars, 
%   IEEE Transactions on Robotics and Automation, vol. 12, pp. 566-580, Aug 1996.
% - Robotics, Vision & Control, Section 5.2.5,
%   P. Corke, Springer 2011.
%
% See also Navigation, PRM, DXform, Dstar, PGraph.

% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

% Peter Corke 8/2009.

%TODO
%   more info to the display method
%   distance metric choice or weightings
%   pass time and model options to the simulation

classdef RRT < Navigation

    properties
        npoints         % number of points to find
        graph           % graph Object representing random nodes

        sim_time        % path simulation time
        kine_model      % simulink model for kinematics

        xrange
        yrange
        
        plant

        speed
        steermax
        vehicle
    end

    methods

        function rrt = RRT(map, vehicle, varargin)
        %RRT.RRT Create an RRT navigation object
        %
        % R = RRT.RRT(MAP, VEH, OPTIONS) is a rapidly exploring tree navigation
        % object for a region with obstacles defined by the map object MAP.
        %
        % R = RRT.RRT() as above but internally creates a Vehicle class object
        % and does not support any MAP or OPTIONS.  For compatibility with
        % RVC book.
        %
        % Options::
        % 'npoints',N    Number of nodes in the tree (default 500)
        % 'time',T       Interval over which to simulate dynamic model toward 
        %                random point (default 0.5s)
        % 'range',R      Specify rectangular bounds
        %                - R scalar; X: -R to +R, Y: -R to +R
        %                - R (1x2); X: -R(1) to +R(1), Y: -R(2) to +R(2)
        %                - R (1x4); X: R(1) to R(2), Y: R(3) to R(4)
        % 'goal',P       Goal position (1x2) or pose (1x3) in workspace
        % 'speed',S      Speed of vehicle [m/s] (default 1)
        % 'steermax',S   Steering angle of vehicle in the range -S to +S [rad] (default 1.2)
        %
        % Notes::
        % - Does not (yet) support obstacles, ie. MAP is ignored but must be given.
        % - 'steermax' selects the range of steering angles that the vehicle will
        %   be asked to track.  If not given the steering angle range of the vehicle
        %   object will be used.
        % - There is no check that the steering range or speed is within the limits
        %   of the vehicle object.
        %
        % Reference::
        % - Robotics, Vision & Control
        %   Peter Corke, Springer 2011.  p102.
        %
        % See also Vehicle.

            % invoke the superclass constructor
            rrt = rrt@Navigation(varargin{:});

            rrt.graph = PGraph(3, 'distance', 'SE2');  % graph of points in SE(2)
            if nargin == 0
                rrt.vehicle = Vehicle([], 'stlim', 1.2);
            else
                % RVC book compatability mode
                rrt.vehicle = vehicle;
            end

            opt.npoints = 500;
            opt.time = 0.5;
            opt.range = 5;
            opt.goal = [0, 0, 0];
            opt.steermax = [];
            opt.speed = 1;
            
            [opt,args] = tb_optparse(opt, varargin);
            rrt.npoints = opt.npoints;
            rrt.sim_time = opt.time;

            switch length(opt.range)
            case 1
                rrt.xrange = [-opt.range opt.range];
                rrt.yrange = [-opt.range opt.range];
            case 2
                rrt.xrange = [-opt.range(1) opt.range(1)];
                rrt.yrange = [-opt.range(2) opt.range(2)];
            case 4
                rrt.xrange = [opt.range(1) opt.range(2)];
                rrt.yrange = [opt.range(3) opt.range(4)];
            otherwise
                error('bad range specified');
            end
            if ~isempty(opt.steermax)
                rrt.steermax = opt.steermax;
            else
                rrt.steermax = rrt.vehicle.alphalim;
            end
            rrt.speed = opt.speed;
            rrt.goal = opt.goal;
        end

        function plan(rrt, varargin)
        %RRT.plan Create a rapidly exploring tree
        %
        % R.plan(OPTIONS) creates the tree roadmap by driving the vehicle
        % model toward random goal points.  The resulting graph is kept
        % within the object.
        %
        % Options::
        % 'goal',P        Goal pose (1x3)
        % 'ntrials',N     Number of path trials (default 50)
        % 'noprogress'    Don't show the progress bar
        % 'samples'       Show progress in a plot of the workspace
        %                 - '.' for each random point x_rand
        %                 - 'o' for the nearest point which is added to the tree
        %                 - red line for the best path
        %
        % Notes::
        % - At each iteration we need to find a vehicle path/control that moves it
        %   from a random point towards a point on the graph.  We sample ntrials of
        %   random steer angles and velocities and choose the one that gets us
        %   closest (computationally slow, since each path has to be integrated
        %   over time).

            opt.progress = true;
            opt.samples = false;
            opt.goal = [];
            opt.ntrials = 50;
            
            opt = tb_optparse(opt, varargin);

            if ~isempty(opt.goal)
                rrt.goal = opt.goal;
            end

            % build a graph over the free space
            rrt.message('create the graph');
            rrt.graph.clear();

            if rrt.verbose
                clf
                %idisp(1-rrt.occgrid, 'ynormal', 'nogui');
                hold on
            end

            % check goal sanity
            if isempty(rrt.goal)
                error('no goal specified');
            end
            switch length(rrt.goal)
            case 2
                rrt.goal = [rrt.goal(:); 0];
            case 3
            otherwise
                error('goal must be 3-vector');
            end

            % add the goal point as the first node
            rrt.graph.add_node(rrt.goal);

            % graphics setup
            if opt.progress
                h = waitbar(0, 'RRT planning...');
            end
            if opt.samples
                clf
                hold on
                xlabel('x'); ylabel('y');
            end

            for j=1:rrt.npoints       % build the tree

                if opt.progress
                    waitbar(j / rrt.npoints);
                end
                
                % Step 3
                % find random state x,y

                % pick a point not in obstacle
                while true
                    xy = rrt.randxy();  % get random coordinate (x,y)

                    % test if lies in the obstacle map (if it exists)
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
                theta = rrt.rand*2*pi;
                xrand = [xy, theta]';
                if opt.samples
                    plot(xy(1), xy(2), '.')
                end

                % Step 4
                % find the existing node closest in state space

                vnear = rrt.graph.closest(xrand);   % nearest vertex
                xnear = rrt.graph.coord(vnear);     % coord of nearest vertex

                rrt.message('xrand (%g, %g) node %d', xy, vnear);

                % Step 5
                % figure how to drive the robot from xnear to xrand
                                
                best = rrt.bestpath(xnear, xrand, opt.ntrials);
                
                xnew = best.path(:,best.k);
                if opt.samples
                    plot(xnew(1), xnew(2), 'o');
                    plot2(best.path', 'r');
                    drawnow
                end

%                 % ensure that the path is collision free
%                 if ~rrt.clearpath(y(:,1:2))
%                     disp('path collision');
%                     continue;
%                 end

                % Step 7,8
                % add xnew to the graph, with an edge from xnear
                v = rrt.graph.add_node(xnew);
                rrt.graph.add_edge(vnear, v);
                
                rrt.graph.setdata(v, best);
            end

            if opt.progress
                close(h)
            end
            rrt.message('graph create done');
        end

        function p_ = path(rrt, xstart, xgoal)
        %RRT.path Find a path between two points
        %
        % X = R.path(START, GOAL) finds a path (Nx3) from state START (1x3) 
        % to the GOAL (1x3).
        %
        % R.path(START, GOAL) as above but plots the path in 3D.  The nodes
        % are shown as circles and the line segments are blue for forward motion
        % and red for backward motion.
        %
        % Notes::
        % - The path starts at the vertex closest to the START state, and ends
        %   at the vertex closest to the GOAL state.  If the tree is sparse this
        %   might be a poor approximation to the desired start and end.
        %
        % See also RRT.plot.

            g = rrt.graph;
            vstart = g.closest(xstart);
            vgoal = g.closest(xgoal);

            % find path through the graph using A* search
            path = g.Astar(vstart, vgoal);
            
            % concatenate the vehicle motion segments
            cpath = [];
            for i = 1:length(path)
                p = path(i);
                data = g.data(p);
                if ~isempty(data)
                    if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                        cpath = [cpath data.path];
                    else
                        cpath = [cpath data.path(:,end:-1:1)];

                    end
                end
            end

            if nargout == 0
                % plot the path
                clf; hold on

                plot2(g.coord(path)', 'o');     % plot the node coordinates
                
                for i = 1:length(path)
                    p = path(i)
                    b = g.data(p);            % get path data for segment
                    
                    % draw segment with direction dependent color
                    if ~isempty(b)
                        % if the vertex has a path leading to it
                        
                        if i >= length(path) || g.edgedir(p, path(i+1)) > 0
                            % positive edge
                            %  draw from prev vertex to end of path
                            seg = [g.coord(path(i-1)) b.path]';
                        else
                            % negative edge
                            %  draw reverse path to next next vertex
                            seg = [  b.path(:,end:-1:1)  g.coord(path(i+1))]';
                        end
                        
                        if b.vel > 0
                            plot2(seg, 'b');
                        else
                            plot2(seg, 'r');
                        end
                    end
                end

                xlabel('x'); ylabel('y'); zlabel('\theta');
                grid
            else
                p_ = cpath';
            end
        end

        function plot(rrt, varargin)
        %RRT.plot Visualize navigation environment
        %
        % R.plot() displays the navigation tree in 3D.

            clf
            rrt.graph.plot('noedges', 'NodeSize', 6, 'NodeFaceColor', 'g', 'NodeEdgeColor', 'g', 'edges');

            hold on
            for i=2:rrt.graph.n
                b = rrt.graph.data(i);
                plot2(b.path(:,1:b.k)')
            end
            xlabel('x'); ylabel('y'); zlabel('\theta');
            grid; hold off
        end

        % required by abstract superclass
        function next(rrt)
        end

        function s = char(rrt)
        %RRT.char  Convert to string
        %
        % R.char() is a string representing the state of the RRT
        % object in human-readable form.
        %
        
            % invoke the superclass char() method
            s = char@Navigation(rrt);

            % add RRT specific stuff information
            s = char(s, sprintf('  region: X %f : %f; Y %f : %f', rrt.xrange, rrt.yrange));
            s = char(s, sprintf('  path time: %f', rrt.sim_time));
            s = char(s, sprintf('  graph size: %d nodes', rrt.npoints));
            s = char(s, char(rrt.graph) );
            if ~isempty(rrt.vehicle)
                s = char(s, char(rrt.vehicle) );
            end
        end
        
        function test(rrt)
            xy = rrt.randxy()
        end


    end % methods


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    P R I V A T E    M E T H O D S
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    methods (Access='protected')

        function best = bestpath(rrt, x0, xg, N)

            % initial and final state as column vectors
            x0 = x0(:); xg = xg(:);

            best.d = Inf;
            for i=1:N   % for multiple trials 
            
                %choose random direction of motion and random steer angle
                if rand > 0.5
                    vel = rrt.speed;
                else
                    vel = -rrt.speed;
                end
                steer = (2*rrt.rand - 1) * rrt.steermax;    % uniformly distributed
                
                % simulate motion of vehicle for this speed and steer angle which 
                % results in a path
                x = rrt.vehicle.run2(rrt.sim_time, x0, vel, steer)';
                
                %% find point on the path closest to xg
                % distance of all path points from goal
                d = colnorm( [bsxfun(@minus, x(1:2,:), xg(1:2)); angdiff(x(3,:), xg(3))] );
                % the closest one
                [dmin,k] = min(d);
                
                % is it the best so far?
                if dmin < best.d
                    % yes it is!  save it and the inputs that led to it
                    best.d = dmin;
                    best.path = x;
                    best.steer = steer;
                    best.vel = vel;
                    best.k = k;
                end
            end 
        end 

        % generate a random coordinate within the working region
        function xy = randxy(rrt)
            xy = rrt.rand(1,2) .* [rrt.xrange(2)-rrt.xrange(1) rrt.yrange(2)-rrt.yrange(1)] + ...
                [rrt.xrange(1) rrt.yrange(1)];
        end

        % test if a path is obstacle free
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


    end % private methods
end % class
