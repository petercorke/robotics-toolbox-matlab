% Astar (A*)
% A* navigation class
% 
% A concrete subclass of the Navigation class that implements the A* navigation algorithm. This 
% provides minimum distance paths that are guaranteed optimal.
% 
% Methods:
% 	plan            Compute the cost map given a goal and map
% 	path            Compute a path to the goal
% 	visualize       Display the obstacle map (deprecated)
% 	plot            Display the obstacle map
% 	costmap_modify 	Modify the costmap
% 	costmap_get     Return the current costmap
% 	costmap_set     Set the current costmap
% 	distancemap_get Set the current distance map
% 	display         Print the parameters in human readable form
% 	char            Convert to string
% 
% Properties:
% costmap           Distance from each point to the goal.
% 
% Example:
% 	load map1           % load map
% 	goal = [50;30];
% 	start = [20;10];
% 	as = Astar(map);	% create Navigation object
% 	as.plan(goal);		% setup costmap for specified goal
% 	as.path(start);		% plan solution path star-goal, animate
% 	P = as.path(start);	% plan solution path star-goal, return path
% Example 2:
% 	goal = [100;100];
% 	start = [1;1];
% 	as = Astar(0);      % create Navigation object with random occupancy grid
% 	as.plan(goal);		% setup costmap for specified goal
% 	as.path(start);		% plan solution path start-goal, animate
% 	P = as.path(start);	% plan solution path start-goal, return path
%     
% Notes
% - Obstacles are represented by Inf in the costmap.
% - The value of each element in the costmap is the shortest distance from the corresponding 
% point in the map to the current goal.
% 
% References
% - A Pareto Front-Based Multiobjective Path Planning Algorithm, A. Lavin.
% - Robotics, Vision & Control, Sec 5.2.2, Peter Corke, Springer, 2011.
% 
% See Also
% Navigation, AstarMOO, AstarPO

% Copyright (C) 1993-2014, by Peter I. Corke, Alexander Lavin
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
%
% The RTB implementation of this algorithm is done by Alexander Lavin.
% http://alexanderlavin.com


% Implementation notes:
%
% All the state is kept in the structure called a
% X is an index into the array of states.
% state pointers are kept as matlab array index rather than row,col format

classdef Astar < Navigation

    properties (SetAccess=private, GetAccess=private)

        % essential world info
        G       % index of goal point
        costmap % world cost map: cost of traversing a cell, obstacle = Inf
        
        % info kept per cell (state):
        b            % backpointer (0 means not set)
        t            % tag: NEW OPEN CLOSED
        
        cost_g       % path distance cost
        cost_h       % path heuristic (state to goal) cost
        cost_f       % composite path cost
        
        % list of open states: 2xN matrix
        %   each open point is a column, row 1 = index of cell, row 2 = k
        openlist
        niter

        changed

        openlist_maxlen     % keep track of maximum length
        quiet

        % tag state values
        NEW = 0;
        OPEN = 1;
        CLOSED = 2;
    end
    
    methods % start of public methods
        % Astar class constructor:
        function as = Astar(world, varargin)
            %Astar.Astar A* constructor
            %
            % AS = Astar(MAP, OPTIONS) is a A* navigation object, and MAP is an
            % occupancy grid, a representation of a planar world as a
            % matrix whose elements are 0 (free space) or 1 (occupied).
            % The occupancy grid is coverted to a costmap with a unit cost
            % for traversing a cell.
            %
            % Options::
            % 'world' = 0 will call for a random occupancy grid to be built
            % 'goal',G      Specify the goal point (2x1)
            % 'metric',M    Specify the distance metric as 'euclidean' (default)
            %               or 'cityblock'.
            % 'inflate',K   Inflate all obstacles by K cells.
            % 'quiet'       Don't display the progress spinner
            %
            % Other options are supported by the Navigation superclass.
            %
            % See also Navigation.Navigation.
            
            % invoke the superclass constructor
            as = as@Navigation(world, varargin{:}); % includes the occgrid

            % options
            opt.quiet = false;
            opt = tb_optparse(opt, varargin);
            as.quiet = opt.quiet;

            % initialize the A* state variables
            as.reset();
            if ~isempty(as.goal)
                as.goal_change();
            end
            as.changed = false;
        end
        
        function reset(as)
            %Astar.reset Reset the planner
            %
            % AS.reset() resets the A* planner.  The next instantiation
            % of AS.plan() will perform a global replan.

            % build the matrices required to hold the state of each cell for A*
            as.b = zeros(size(as.occgrid), 'uint32');     % backpointers
            as.t = zeros(size(as.occgrid), 'uint8');      % tags, all NEW=0
            as.cost_g = Inf*ones(size(as.costmap));       % path cost estimate
            as.openlist = zeros(2,0);                     % the open list, one column per point
            
            as.openlist_maxlen = -Inf;
        end
        
        function goal_change(as)
            %Astar.goal_change Changes to costlayers due to new goal
            %position
            if isempty(as.b)
                return;
            end
            goal = as.goal;
            as.calcHeuristic(as.occgrid, as.goal);

            % keep goal in index rather than row,col format
            as.G = sub2ind(size(as.occgrid), goal(2), goal(1));
            as.INSERT(as.G, 0, 'goalset');
        end
        
        function s = char(as)
            %Astar.char Convert navigation object to string
            %
            % AS.char() is a string representing the state of the Astar
            % object in human-readable form.
            %
            % See also Astar.display, Navigation.char.
 
            % most of the work is done by the superclass
            s = char@Navigation(as);
        end

        function plot(as, varargin)
            %Astar.plot Visualize navigation environment
            %
            % AS.plot() displays the occupancy grid and the goal distance
            % in a new figure.  The goal distance is shown by intensity which
            % increases with distance from the goal.  Obstacles are overlaid
            % and shown in red.
            %
            % AS.plot(P) as above but also overlays a path given by the set
            % of points P (Mx2).
            %
            % See also Navigation.plot.
            
            plot@Navigation(as, 'distance', as.cost_h, varargin{:});
        end

       % invoked by Navigation.step
        function n = next(as, current)
            % Backpropagate from goal to start
            % Return [col;row] of previous step
            if as.changed
                error('Cost map has changed, replan');
            end
            X = sub2ind(size(as.occgrid), current(2), current(1));
            X = as.b(X); % set X as the backpointer of X
            if X == 0
                n = []; % goal (no further backpointer)
            else
                [r,c] = ind2sub(size(as.costmap), X);
                n = [c;r];
            end
        end        
        
        function plan(as, goal)
            %Astar.plan Prep the grid for planning.
            %
            % AS.plan() updates AS with a costmap of distance to the
            % goal from every non-obstacle point in the map.  The goal is
            % as specified to the constructor.
            %
            % AS.plan(GOAL) as above but uses the specified goal.
            
            if nargin > 1
                as.goal = goal; % invokes superclass method set.goal()
            end
            if isempty(as.goal)
                error('must specify a goal point');
            end
            
            % initialize the distance and composite cost layers
            as.cost_g = zeros(size(as.occgrid));
            as.cost_f = zeros(size(as.occgrid));
            % fill heuristic layer
            as.calcHeuristic(as.occgrid, as.goal);
            % assign values to the distance cost layer, set as AS.costmap
            as.occgrid2costmap(as.occgrid);
        end
        
        function P = path(as, start)
        %AS.path Find a path between two points
        %
        % AS.path(START) finds and displays a path from START to GOAL
        % which is overlaid on the occupancy grid.
        %
        % P = AS.path(START) returns the path (2xM) from START to GOAL.
            if nargin < 1
                error('must specify start state');
            end

            % invoke the superclass path function, which iterates on our
            % next method
            start = [start; 1]; % Specifies backpropogation for NAV.path()
            if nargout == 0
                path@Navigation(as, start);
            else
                P = path@Navigation(as, start);
            end
        end
        
        % Handler invoked by Navigation.path() to start the navigation process
        % Calculate the solution path -> AS.localPath
        function navigate_init(as, start)
            as.openlist = zeros(2,0); % make sure openlist is empty
            % begin with the start node
            start = sub2ind(size(as.costmap), start(2), start(1));
            as.openlist(1,1) = start;
            as.t(start) = as.OPEN;
            as.cost_g(start) = 0;
            as.niter = 0; flag = 0; % while loop initializations
            % Plan the A* path
            while ~isempty(as.openlist) % L4; while openlist is not empty
                % Check openlist, choose expansion state: 
                X = as.MIN_STATE(); % L5; minimum state on the open list
                if isempty(X)                               % L2
                    display('Search failed')
                    return;
                end
                as.DELETE(X); % L6; remove state from the open list
                
                % Track path expansion:
                as.niter = as.niter + 1;
                
                if ~as.quiet && mod(as.niter, 20) == 0 % i.e. spinner every 20 iterations
                    as.spinner();
                end
                
                % Populate the openlist:
                for Y=as.neighbors(X) % L7,8; check node's 8 neighbors
                    if(Y==as.G) % L9; check for goal
                        as.b(Y) = X;
                        as.cost_g(X) = as.cost_g(X) + as.dc(X,Y);
                        as.cost_f(X) = as.cost_f(X) + as.dc(X,Y);
                        flag = 1; % flag for goal
                        break;
                    end
                    if as.t(Y)==as.NEW && as.costmap(Y)~=Inf % hasn't been checked and isn't an obstacle
                        % INSERT w/ the distance to X plus the distance X
                        % to Y, set the backpointer of Y as X
                        as.b(Y) = X;
%                         cost = as.cost_g(X) + as.dc(X,Y) + as.cost_h(Y); % this is the cost of Y
%                         as.INSERT(Y, cost, ''); % stores cost into cost_f(Y)
                        D = as.cost_g(X) + as.dc(X,Y);
                        as.INSERT(Y, D, ''); % stores cost into cost_f(Y)
                    end
                end                
                if as.verbose
                    disp(' ')
                end
                if flag==1 % goal
                    break;
                end
            end
            if ~as.quiet
                fprintf('\r');
            end
            as.changed = false;
        end
        
        function c = heurstic_get(as)
        %Astar.heuristice_get Get the current heuristic map
        %
        % C = AS.heuristice_get() is the current heuristic map.  This map is the same size
        % as the occupancy grid and the value of each element is the shortest distance 
        % from the corresponding point in the map to the current goal.  It is computed
        % by Astar.plan.
        %
        % See also Astar.plan.
            c = as.cost_h;
        end

        function c = costmap_get(as)
        %Astar.costmap_get Get the current costmap
        %
        % C = AS.costmap_get() is the current costmap.  The cost map is the same size
        % as the occupancy grid and the value of each element represents the cost
        % of traversing the cell.  It is autogenerated by the class constructor from
        % the occupancy grid such that:
        % - free cell (occupancy 0) has a cost of 1
        % - occupied cell (occupancy >0) has a cost of Inf
        %
        % See also Astar.costmap_set, Astar.costmap_modify.

            c = as.costmap;
        end

        function costmap_set(as, costmap)
        %Astar.costmap_set Set the current costmap
        %
        % AS.costmap_set(C) sets the current costmap.  The cost map is the same size
        % as the occupancy grid and the value of each element represents the cost
        % of traversing the cell.  A high value indicates that the cell is more costly
        % (difficult) to traverese.  A value of Inf indicates an obstacle.
        %
        % Notes:
        % - After the cost map is changed the path should be replanned by 
        %   calling AS.plan(). 
        %
        % See also Astar.costmap_get, Astar.costmap_modify.
            if ~all(size(costmap) == size(as.occgrid))
                error('costmap must be same size as occupancy grid');
            end
            as.costmap = costmap;
            as.changed = true;
            end

        function costmap_modify(as, point, newcost)
        %Astar.costmap_modify Modify cost map
        %
        % AS.costmap_modify(P, NEW) modifies the cost map at P=[X,Y] to
        % have the value NEW.  If P (2xM) and NEW (1xM) then the cost of
        % the points defined by the columns of P are set to the corresponding
        % elements of NEW.
        %
        % Notes::
        % - After one or more point costs have been updated the path
        %   should be replanned by calling DS.plan().
        %
        % See also Astar.costmap_set, Astar.costmap_get.

            if numel(point) == 2
                % for case of single point ensure it is a column vector
                point = point(:);
            end
            if numcols(point) ~= numcols(newcost)
                error('number of columns in point must match columns in newcost');
            end
            for i=1:numcols(point)
                X = sub2ind(size(as.costmap), point(2,i), point(1,i));
                as.costmap(X) = newcost(i);
            end
            if as.t(X) == as.CLOSED
                as.INSERT(X, as.cost_g(X), 'modifycost');
            end
            as.changed = true;
        end 
        
    end % end of public methods
    
    methods (Access=protected) % start of private methods
        
        function occgrid2costmap(as, og, cost)
            if nargin < 3
                cost = 1; % cost to traverse each cell
            end
            as.costmap = og;
            as.costmap(as.costmap==1) = Inf; % occupied cells -> infinite path cost
            as.costmap(as.costmap==0) = cost; % unoccupied cells -> path cost
        end
        
        function calcHeuristic(as, grid, goal)
            as.cost_h=zeros(size(grid));
            for ii=1:size(grid,1)
                for jj=1:size(grid,2)
                    as.cost_h(ii,jj)=sqrt((ii-goal(1))^2+(jj-goal(2))^2);
                end
            end
        end
        
        function INSERT(as, X, D, where)
            % X is new node with the cost already calculated into the
            % variable cost.
            
            % where is for diagnostic purposes only
            if nargin>2
                as.message('insert (%s) %d = %f\n', where, X, D);
            end
            
            i = find(as.openlist(1,:) == X);
            if length(i) > 1
                error('A*:INSERT: state in open list %d times', X);
            end

            if as.t(X) == as.NEW
                k_new = D + as.cost_h(X);
            elseif as.t(X) == as.OPEN
                % L13: If a node with same position as successor is in the
                % OPEN list & has a lower f than successor, then skip this
                % successor.
                k_new = min(as.openlist(2,i), (D+as.cost_h(X)));
            elseif as.t(X) == as.CLOSED
                % L14: If a node with same position as successor is in the
                % CLOSED list & has a lower f than successor, then skip this
                % successor.
                k_new = min(as.cost_f(X), (D+as.cost_h(X)));
            end
            % add a new column to the open list
            as.openlist = [as.openlist [X; k_new]];

            % keep track of the max length of the openlist:
            if numcols(as.openlist) > as.openlist_maxlen
                as.openlist_maxlen = numcols(as.openlist);
            end

            % Set state variables for X:
            as.cost_g(X) = D;
            as.cost_f(X) = k_new;
            as.t(X) = as.OPEN;
        end

        function DELETE(as, X)
            as.message('delete %d\n', X);
            i = find(as.openlist(1,:) == X);
            if length(i) ~= 1
                error('D*:DELETE: state %d doesnt exist', X);
            end
            as.openlist(:,i) = []; % remove the column
            as.t(X) = as.CLOSED;
        end
        
        % return the index of the open state with the smallest cost value
        function ms = MIN_STATE(as)
            if isempty(as.openlist)
                ms = [];
            else
                % find the minimum-cost node on the openlist
                [~,i] = min(as.openlist(2,:));
                % return its index
                ms = as.openlist(1,i);
            end
        end
        
        % return the distance cost of moving from state X to state Y
        function cost = dc(as, X, Y)
            [r,c] = ind2sub(size(as.costmap), [X; Y]);
            dist = sqrt(sum(diff([r c]).^2));
            dcost = (as.costmap(X) + as.costmap(Y))/2;

            cost = dist * dcost;
        end

        % return index of neighbor states (max 8) as a row vector
        function Y = neighbors(as, X)
            dims = size(as.costmap);
            [r,c] = ind2sub(dims, X);

            % list of 8-way neighbors:
            Y = [r-1 r-1 r-1 r r  r+1 r+1 r+1; c-1 c c+1 c-1 c+1 c-1 c c+1];
            % only use neighbors w/in grid bounds...
            k = (min(Y)>0) & (Y(1,:)<=dims(1)) & (Y(2,:)<=dims(2));
            Y = Y(:,k);
            Y = sub2ind(dims, Y(1,:)', Y(2,:)')';
        end

    end % end of private methods
end
