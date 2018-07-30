%PRM Probabilistic RoadMap navigation class
%
% A concrete subclass of the abstract Navigation class that implements the
% probabilistic roadmap navigation algorithm over an occupancy grid.  This
% performs goal independent planning of roadmaps, and at the query stage
% finds paths between specific start and goal points.
%
% Methods::
%  PRM          Constructor
%  plan         Compute the roadmap
%  query        Find a path
%  plot         Display the obstacle map
%  display      Display the parameters in human readable form
%  char         Convert to string
%
% Example::
%        load map1              % load map
%        goal = [50,30];        % goal point
%        start = [20, 10];      % start point
%        prm = PRM(map);        % create navigation object
%        prm.plan()             % create roadmaps
%        prm.query(start, goal)  % animate path from this start location
%
% References::
%
% - Probabilistic roadmaps for path planning in high dimensional configuration spaces,
%   L. Kavraki, P. Svestka, J. Latombe, and M. Overmars, 
%   IEEE Transactions on Robotics and Automation, vol. 12, pp. 566-580, Aug 1996.
% - Robotics, Vision & Control, Section 5.2.4,
%   P. Corke, Springer 2011.
%
% See also Navigation, DXform, Dstar, PGraph.



% Copyright (C) 1993-2017, by Peter I. Corke
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

classdef PRM < Navigation

    properties
        npoints         % number of sample points
        npoints0
        distthresh      % distance threshold, links between vertices
                        % must be less than this.
        distthresh0     % distance threshold set by constructor option
        graph           % graph Object representing random nodes

        vgoal           % index of vertex closest to goal
        vstart          % index of vertex closest to start
        localGoal       % next vertex on the roadmap
        localPath       % set of points along path to next vertex
        vpath           % list of vertices between start and goal
        gpath
    end

    methods

        % constructor
        function prm = PRM(varargin)
            %PRM.PRM Create a PRM navigation object
            %
            % P = PRM(MAP, options) is a probabilistic roadmap navigation
            % object, and MAP is an occupancy grid, a representation of a
            % planar world as a matrix whose elements are 0 (free space) or 1
            % (occupied).
            %
            % Options::
            %  'npoints',N      Number of sample points (default 100)
            %  'distthresh',D   Distance threshold, edges only connect vertices closer 
            %                   than D (default 0.3 max(size(occgrid)))
            %
            % Other options are supported by the Navigation superclass.
            %
            % See also Navigation.Navigation.


            % invoke the superclass constructor, it handles some options
            prm = prm@Navigation(varargin{:});

            % create an empty 2D graph
            prm.graph = PGraph(2);

            % parse out PRM specific options and save in the navigation object
            opt.npoints = 100;
            opt.distthresh = 0.3*max(size(prm.occgridnav));
            [opt,args] = tb_optparse(opt, varargin);
            prm.npoints0 = opt.npoints;
            prm.distthresh0 = opt.distthresh;
        end

        function plan(prm, varargin)
        %PRM.plan Create a probabilistic roadmap
        %
        % P.plan(OPTIONS) creates the probabilistic roadmap by randomly
        % sampling the free space in the map and building a graph with
        % edges connecting close points.  The resulting graph is kept
        % within the object.
        %
        % Options::
        %  'npoints',N      Number of sample points (default is set by constructor)
        %  'distthresh',D   Distance threshold, edges only connect vertices closer 
        %                   than D (default set by constructor)
        %  'movie',M        make a movie of the PRM planning

        % build a graph over the free space
            prm.message('create the graph');
            
            opt.npoints = prm.npoints0;
            opt.distthresh = prm.distthresh0;  % default is constructor value
            opt.animate = false;
            opt.movie = [];

            opt = tb_optparse(opt, varargin);
            
            prm.npoints = opt.npoints;
            prm.distthresh = opt.distthresh;  % actual value used is constructor value overridden here
            

            prm.graph.clear();  % empty the graph
            prm.vpath = [];
            create_roadmap(prm, opt);  % build the graph
        end
        
        function pp = query(prm, start, goal)
        %PRM.query Find a path between two points
        %
        % P.query(START, GOAL) finds a path (Mx2) from START to GOAL.
        %
        
            if prm.graph.n == 0
                error('RTB:PRM:noplan', 'query: no plan: run the planner');
            end
            checkquery(prm, start, goal)
            
            % find the vertex closest to the goal
            prm.vgoal = prm.closest(prm.goal);
            if isempty(prm.vgoal)
                error('RTB:PRM:nopath', 'plan: no path roadmap -> goal: rerun the planner');
            end
            
            % find the vertex closest to the start
            prm.vstart = prm.closest(prm.start);
            if isempty(prm.vstart)
                error('RTB:PRM:nopath', 'plan: no path start -> roadmap: rerun the planner');
            end
            
            % find a path through the graph

            prm.vpath = prm.graph.Astar(prm.vstart, prm.vgoal);

            % the path is a list of nodes from vstart to vgoal
            % discard the first vertex, since we plan a local path to it
            prm.gpath = prm.vpath;
            prm.gpath = prm.gpath(2:end);

            if nargout > 0
                pp = [prm.start prm.graph.coord(prm.vpath) prm.goal]';
            end
       
        end

        function c = closest(prm, vertex, vcomponent)
            
            % find a node close to v that is:
            %  - closest
            %  - in the same component
            %  - free straight line path
                            if nargin > 2
            component = prm.graph.component(vcomponent);
                            end
            [d,v] = prm.graph.distances(vertex);
            c = [];
            
            % test neighbours in order of increasing distance and check for a clear
            % path
            for i=1:length(d)
                if nargin > 2
                    if prm.graph.component(v(i)) ~= component
                        continue; % not connected
                    end
                end
                if ~prm.testpath(vertex, prm.graph.coord(v(i)))
                    continue; % no path
                end
                c = v(i);
                break
            end
        end
        
        % Handler invoked by Navigation.path() to start the navigation process
        %
        %   - find a path through the graph
        %   - determine vertices closest to start and goal
        %   - find path to first vertex


        % Invoked for each step on the path by path() method.
        function n = next(prm, p)

            if all(p(:) == prm.goal)
                n = [];     % signal that we've arrived
                return;
            end

            % we take the next point from the localPath
            if numrows(prm.localPath) == 0
                % local path is consumed, move to next vertex
                if isempty(prm.gpath)
                    % we have arrived at the goal vertex
                    % make the path from this vertex to the goal coordinate
                    prm.localPath = bresenham(p, prm.goal);
                    prm.localPath = prm.localPath(2:end,:);
                    prm.localGoal = [];
                else
                    % set local goal to next vertex in gpath and remove it from the list
                    prm.localGoal = prm.gpath(1);
                    prm.gpath = prm.gpath(2:end);

                    % compute local path to the next vertex
                    prm.localPath = bresenham(p, prm.graph.coord(prm.localGoal));
                    prm.localPath = prm.localPath(2:end,:);
                    prm.graph.highlight_node(prm.localGoal);
                end
            end

            n = prm.localPath(1,:)';     % take the first point
            prm.localPath = prm.localPath(2:end,:); % and remove from the path
        end

        function s = char(prm)
        %PRM.char  Convert to string
        %
        % P.char() is a string representing the state of the PRM
        % object in human-readable form.
        %
        % See also PRM.display.


            % invoke the superclass char() method
            s = char@Navigation(prm);

            % add PRM specific stuff information
            s = char(s, sprintf('  graph size: %d', prm.npoints));
            s = char(s, sprintf('  dist thresh: %g', prm.distthresh));
            s = char(s, char(prm.graph) );
        end
        
        
        function plot(prm, varargin)
        %PRM.plot Visualize navigation environment
        %
        % P.plot() displays the roadmap and the occupancy grid.
        %
        % Options::
        %  'goal'            Superimpose the goal position if set
        %  'nooverlay'       Don't overlay the PRM graph
        %
        % Notes::
        % - If a query has been made then the path will be shown.
        % - Goal and start locations are kept within the object.
            
            opt.overlay = true;
            opt.nodes = true;
            [opt,args] = tb_optparse(opt, varargin);
            
            % display the occgrid
            plot@Navigation(prm, args{:});
            
            if opt.overlay
                hold on
                prm.graph.plot('componentcolor');
                
                if opt.nodes && ~isempty(prm.vpath)
                    prm.graph.highlight_path(prm.vpath, ...
                        'NodeFaceColor', 'y', ...
                        'NodeEdgeColor', 'k', ...
                        'EdgeColor', 'k', ...
                        'EdgeThickness', 2 ...
                        )
                    
                    v0 = prm.vpath(1);
                    p0 = prm.graph.coord(v0);
                    plot([prm.start(1) p0(1)], [prm.start(2) p0(2)], 'Color', 'k', ...
                        'LineWidth', 1.5);
                    vf = prm.vpath(end);
                    pf = prm.graph.coord(vf);
                    
                    plot([prm.goal(1) pf(1)], [prm.goal(2) pf(2)], 'Color', 'k', ...
                        'LineWidth', 1.5);
                end
            end
            
%             % get the superclass to plot the path
%             if nargin > 1
%                 plot@Navigation(prm, varargin{:});
%             end
            
            hold off
            set(gcf, 'Color', [1 1 1])
        end

    end % method

    methods (Access='protected')
    % private methods
        % create the roadmap
        function create_roadmap(prm, opt)

            a = Animate(opt.movie, 'fps', 5);
            
            for j=1:prm.npoints
                % pick a point not in obstacle
                while true
                    x = prm.randi(numcols(prm.occgrid));
                    y = prm.randi(numrows(prm.occgrid));
                    if ~prm.isoccupied([x y])
                        break; % free cell
                    end
                end
                new = [x; y];
                
                % add it to the graph
                vnew = prm.graph.add_node(new);

                % find the closest node already in the graph
                [d,v] = prm.graph.distances(new);
                
                % test neighbours in order of increasing distance and check for a clear
                % path
                for i=1:length(d)
                    if d(i) > prm.distthresh
                        continue; % it's too far
                    end
                    if ~prm.testpath(new, prm.graph.coord(v(i)))
                        continue; % no path
                    end
                    
                    % add an edge from the found node to new
                    prm.graph.add_edge(v(i), vnew);
                end
                
                if opt.animate || ~isempty(opt.movie)
                    prm.plot()
                    if ~isempty(opt.movie) 
                        a.add();
                    else
                        pause(1)
                    end
                end
            end
        end

        % test the path from p1 to p2 is entirely in free space
        function c = testpath(prm, p1, p2)
            p = bresenham(p1, p2);

            for pp=p'
                if prm.isoccupied(pp)
                    c = false;
                    return;
                end
            end
            c = true;
        end


    end % private methods

end % classdef
