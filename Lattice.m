%Lattice Lattice planner navigation class
%
% A concrete subclass of the abstract Navigation class that implements the
% lattice planner navigation algorithm over an occupancy grid.  This
% performs goal independent planning of kinematically feasible paths.
%
% Methods::
%  Lattice      Constructor
%  plan         Compute the roadmap
%  query        Find a path 
%  plot         Display the obstacle map
%  display      Display the parameters in human readable form
%  char         Convert to string
%
% Properties (read only)::
%  graph        A PGraph object describign the tree
%
% Example::
%
%        lp = Lattice();                    % create navigation object
%        lp.plan('iterations', 8)           % create roadmaps
%        lp.query( [1 2 pi/2], [2 -2 0] )   % find path
%        lp.plot();                         % plot the path
%
% References::
%
% - Robotics, Vision & Control, Section 5.2.4,
%   P. Corke, Springer 2016.
%
%
% See also Navigation, DXform, Dstar, PGraph.


% Notes::
% - The lattice is stored as a 3D PGraph object with coordinates x,y,theta
%   where theta is stored as a multiple of pi/2.  This was probably a bad
%   design decision, it complicates the code a lot.
% - Using the Lattice distance metric in PGraph gives different A* results,
%   valid path, same cost, just different.  Blah.


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

classdef Lattice < Navigation
    
    properties
        iterations         % number of iterations
        cost      % segment cost

        % must be less than this.
        
        graph           % graph Object representing random nodes
        
        vgoal           % index of vertex closest to goal
        vstart          % index of vertex closest to start
        localGoal       % next vertex on the roadmap
        localPath       % set of points along path to next vertex
        vpath           % list of vertices between start and goal
        grid
        root
    end
    
    methods
        
        % constructor
        function lp = Lattice(varargin)
            %Lattice.Lattice Create a Lattice navigation object
            %
            % P = Lattice(MAP, options) is a probabilistic roadmap navigation
            % object, and MAP is an occupancy grid, a representation of a
            % planar world as a matrix whose elements are 0 (free space) or 1
            % (occupied).
            %
            % Options::
            % 'grid',G         Grid spacing in X and Y (default 1)
            % 'root',R         Root coordinate of the lattice (2x1) (default [0,0])
            % 'iterations',N   Number of sample points (default Inf)
            % 'cost',C         Cost for straight, left, right (default [1,1,1])
            % 'inflate',K      Inflate all obstacles by K cells.
            %
            % Other options are supported by the Navigation superclass.
            %
            % Notes::
            % - Iterates until the area defined by the map is covered.
            %
            % See also Navigation.Navigation.
            
            
            % invoke the superclass constructor, it handles some options
            lp = lp@Navigation(varargin{:});
            
            % create an empty graph over SE2
            lp.graph = PGraph(3, 'distance', 'SE2');
            
            % parse out Lattice specific options and save in the navigation object
            opt.grid = 1;
            opt.root = [0 0 0]';
            opt.iterations = Inf;
            opt.cost = [1 1 1];
            [lp,args] = tb_optparse(opt, varargin, lp); 
        end
        
        function plan(lp, varargin)
            %Lattice.plan Create a lattice plan
            %
            % P.plan(OPTIONS) creates the lattice by iteratively building a tree of
            % possible paths.  The resulting graph is kept within the object.
            %
            % Options::
            % 'iterations',N   Number of sample points (default Inf)
            % 'cost',C         Cost for straight, left, right (default [1,1,1])
            %
            % Default parameter values come from the constructor
            
            opt.iterations = lp.iterations;
            opt.cost = lp.cost;

            [opt,args] = tb_optparse(opt, varargin);
            
            if isempty(lp.occgridnav) && isinf(opt.iterations)
                error('RTB:Lattice:badarg', 'If no occupancy grid given then iterations must be finite');
            end
            
            lp.iterations = opt.iterations;
            lp.cost = opt.cost;
            
            % check root node sanity
            if isempty(lp.root)
                error('no root node specified');
            end
            switch length(lp.root)
                case 2
                    lp.root = [lp.root(:); 0];
                case 3
                    lp.root = lp.root(:);
                otherwise
                    error('root must be 2- or 3-vector');
            end
            
            if lp.isoccupied(lp.root(1:2))
                error('root node cell is occupied')
            end
            
            % build a graph over the free space
            lp.message('create the graph');
            
            lp.graph.clear();  % empty the graph
            create_lattice(lp);  % build the graph
            fprintf('%d nodes created\n', lp.graph.n);
        end
        
        
        function pp = query(lp, start, goal)
            %Lattice.query Find a path between two poses
            %
            % P.query(START, GOAL) finds a path (Nx3) from pose START (1x3) 
            % to pose GOAL (1x3).  The pose is expressed as [X,Y,THETA]. 
            %
            
            if nargin < 3
                error('must specify start and goal');
            end
            
            % set the goal coordinate
            lp.goal = goal;
            lp.start = start;
            
            % convert angles to multiple of pi2
            start(3) = round(start(3)*2/pi);
            goal(3) = round(goal(3)*2/pi);
            
            lp.vstart = lp.graph.closest(start, 0.5);
            lp.vgoal = lp.graph.closest(goal, 0.5);
            
            if isempty(lp.vstart)
                error('Lattice:badarg', 'start configuration not in lattice');
            end
            if isempty(lp.vgoal)
                error('Lattice:badarg', 'goal configuration not in lattice');
            end
            
            % find path through the graph using A* search
            [lp.vpath,cost] = lp.graph.Astar(lp.vstart, lp.vgoal, 'directed');
            fprintf('A* path cost %g\n', cost);
            
            p = lp.graph.coord(lp.vpath);
            
            if nargout > 0
                pp = p';
                pp(:,3) = angdiff( pp(:,3) * pi/2 );
            end
        end
        
        
        % Handler invoked by Navigation.path() to start the navigation process
        %
        %   - find a path through the graph
        %   - determine vertices closest to start and goal
        %   - find path to first vertex

        
        % Invoked for each step on the path by path() method.
        function n = next(lp, p)
            
        end
        
        function s = char(lp)
            %Lattice.char  Convert to string
            %
            % P.char() is a string representing the state of the Lattice
            % object in human-readable form.
            %
            % See also Lattice.display.
            
            
            % invoke the superclass char() method
            s = char@Navigation(lp);
            
            % add Lattice specific stuff information
            s = char(s, sprintf('  grid spacing: %d', lp.grid));
            s = char(s, sprintf('  costs [%d,%d,%d]', lp.cost));
            s = char(s, sprintf('  iterations %d', lp.iterations));
            s = char(s, sprintf(' Graph:'));
            s = char(s, char(lp.graph) );
        end
        
        
        function plot(lp, varargin)
            %Lattice.plot Visualize navigation environment
            %
            % P.plot() displays the occupancy grid with an optional distance field.
            %
            % Options::
            %  'goal'            Superimpose the goal position if set
            %  'nooverlay'       Don't overlay the Lattice graph
            
            % get standard stuff done by the superclass
            plot@Navigation(lp, varargin{:});
            
            opt.nooverlay = false;
            [opt,args] = tb_optparse(opt, varargin);
            
            if ~opt.nooverlay
                hold on
                lp.showlattice(varargin{:});
                hold off
            end
            
            if ~isempty(lp.vpath)
                % highlight the path
                hold on
                lp.highlight(args{:});
                hold off
            end
            
            grid on
        end
        
%         function path = animate(lp, varargin)
%             path = [];
%             for k=1:length(lp.vpath)-1
%                 v1 = lp.vpath(k);
%                 v2 = lp.vpath(k+1);
% 
%                 seg = drivearc(lp, [v1, v2], 10);
%                 path = [path seg(:,1:end-1)];
%             end
%             path = [path seg(:,end)];
% 
%         end
        
    end % method
    
    methods (Access='protected')
        % private methods
        
        % create the lattice
        function create_lattice(lp)
            
            % add the root node
            root = lp.graph.add_node( lp.root );
            
            % possible destinations in root node frame
            %   x direction is forward
            %   orientation represented by integer 0-3 representing multiples of pi/2
            d = lp.grid;
            
            destinations = [
                d  d  d   % x
                0  d -d   % y
                0  1  3   % theta *pi/2
                ];
            
            % now we iterate, repeating this patter at each leaf node
            iteration = 1;
            while iteration <= lp.iterations
                additions = 0;
                
                for node = find(lp.graph.connectivity_out == 0) % foreach leaf node
                    
                    % get the pose of this node
                    pose = lp.graph.coord(node);
                    xys = pose(1:2); heading = pose(3);
                    
                    % transform the motion directions to this pose and b
                    xy = bsxfun(@plus, xys, homtrans(rot2(heading*pi/2), destinations(1:2,:)));
                    theta = mod(heading+destinations(3,:), 4);
                    newDestinations = [xy; theta];
                    
                    % now add paths to these new poses
                    for i=1:numcols(destinations)
                        % check to see if a node for this pose already exists
                        v = lp.graph.closest(newDestinations(:,i), 0.5);
                        if isempty(v)
                            %node doesn't exist
                            if ~lp.isoccupied(newDestinations(1:2,i))
                                % it's not occupied
                                % add a new node and an edge
                                nv = lp.graph.add_node( newDestinations(:,i), node, lp.cost(i));
                                lp.graph.add_edge(node, nv, lp.cost(i));
                                additions = additions + 1;
                            end
                        else
                            % node already exists, add an edge
                            lp.graph.add_edge(node, v, lp.cost(i));
                            additions = additions + 1;
                        end
                    end
                end
                iteration = iteration + 1;
                if additions == 0
                    break;  % no more nodes can be added to the space
                end
            end
        end
        
        % Display the lattice, possible arcs, and start/goal markers if relevant
        function showlattice(lp, varargin)
            
            lineopt = {'Linewidth', 0.2, 'Color', [0.5 0.5 0.5]};
            markeropt = {'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b'};
            
            p = lp.graph.coord();
            th = p(3,:);
            th(th == 3) = -1;
            plot3(p(1,:), p(2,:), th*pi/2, markeropt{:});
            xlabel('x'); ylabel('y'); zlabel('\theta')
            grid on
            hold on
            plot3(lp.root(1), lp.root(2), lp.root(3), 'ko', 'MarkerSize', 8);
            view(0,90);
            axis equal
            rotate3d
            
            % draw the lattice
            for e=1:lp.graph.ne
                v = lp.graph.vertices(e);  % get the vertices of the edge
                drawarc(lp, v, lineopt);
            end
        end
        
        function highlight(lp, p)
            
            if nargin > 1
                assert(numcols(p)==3, 'path must have 3 columns');
                for i=1:numrows(p)
                    
                    vpath(i) = lp.graph.closest(p(i,:) );
                end
            else
                vpath = lp.vpath;
            end
            
            % highlight the path
            for k=1:length(vpath)-1
                v1 = vpath(k);
                v2 = vpath(k+1);
                drawarc(lp, [v1, v2], {'Linewidth', 3, 'Color', 'r'});
            end
        end
        
        % draw an arc
        function drawarc(lp, v, lineOpts)
            g = lp.graph;
            
            % use lower resolution if lots of arcs
            if lp.iterations < 4
                narc = 20;
            elseif lp.iterations < 10
                narc = 10;
            else
                narc = 5;
            end
            
            v1 = v(1); v2 = v(2);
            p1 = g.coord(v1);
            p2 = g.coord(v2);
            
            % frame {N} is start of the arc
            theta = p1(3)*pi/2;  % {0} -> {N}
            T_0N = SE2(p1(1:2), theta);
            
            dest = round( T_0N.inv * p2(1:2) );  % in {N}
            
            if dest(2) == 0
                % no heading change, straight line segment
                th = [p1(3) p2(3)];
                th(th == 3) = -1;
                plot3([p1(1) p2(1)], [p1(2) p2(2)], th*pi/2, lineOpts{:});
            else
                % curved segment
                c = T_0N * [0 dest(2)]';
                
                th = ( linspace(-dest(2)/lp.grid, 0, narc) + p1(3) )*pi/2;
                
                x = lp.grid*cos(th) + c(1);
                y = lp.grid*sin(th) + c(2);
                
                
                th0 = p1(3);
                th0(th0==3) = -1;
                thf = p2(3);
                thf(thf==3) = -1;
                plot3(x, y, linspace(th0, thf, narc)*pi/2, lineOpts{:});
            end
        end
        
%         % this doesn't work quite properly...
%         function path = drivearc(lp, v, narc)
%             g = lp.graph;
%             
%             
%             v1 = v(1); v2 = v(2);
%             p1 = g.coord(v1);  p1(3) = p1(3)*pi/2;
%             p2 = g.coord(v2);  p2(3) = p2(3)*pi/2;
%             
%             path = [];
%             
%             % frame {N} is start of the arc
%             theta = p1(3);  % {0} -> {N}
%             T_0N = SE2(p1(1:2), theta);
%             
%             dest = round( T_0N.inv * p2(1:2) );  % in {N}
%             
%             if dest(2) == 0
%                 % no heading change, straight line segment
%                 
%                 for s=linspace(0, 1, narc)
%                     path = [path (1-s)*p1 + s*p2];
%                 end
%             else
%                 % curved segment
%                 c = T_0N * [0 dest(2)]';
%                 
%                 th = ( linspace(-dest(2)/lp.grid, 0, narc) + p1(3) );
%                 
%                 x = lp.grid*cos(th) + c(1);
%                 y = lp.grid*sin(th) + c(2);
%                 
%                 
%                 th0 = p1(3);
% % %                 th0(th0==3) = -1;
%                 thf = p2(3);
% % %                 thf(thf==3) = -1;
%                 path = [path [x; y; linspace(th0, angdiff(thf,th0)+th0, narc)] ];
%             end
%         end

        
    end % private methods
    
end % classdef
