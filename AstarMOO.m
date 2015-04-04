% AstarMOO A*-MOO navigation class
% 
% A concrete subclass of the Navigation class that implements the A*
% navigation algorithm for multiobjective optimization (MOO) - i.e.
% optimizes over several objectives/criteria.
% 
% Methods:
%     plan              Compute the cost map given a goal and map
%     path              Compute a path to the goal
%     visualize         Display the obstacle map (deprecated)
%     plot              Display the obstacle map
%     costmap_modify    Modify the costmap
%     costmap_get       Return the current costmap
%     costmap_set       Set the current costmap
%     distancemap_get   Set the current distance map
%     heuristic_get     Get the current heuristic map
%     display           Print the parameters in human readable form
%     char              Convert to string
% 
% Properties:
% TBD
% 
% Example::
%
%         load map1           % load map
%         goal = [50;30];
%         start = [20;10];
%         as = AstarMOO(map);    % create Navigation object
%         as.plan(goal,2);       % setup costmap for specified goal
%         as.path(start);        % plan solution path star-goal, animate
%         P = as.path(start);    % plan solution path star-goal, return path
%
% Example 2:
%         goal = [100;100];
%         start = [1;1];
%         as = AstarMOO(0);   % create Navigation object with random occupancy grid
%         as.addCost(1,L);    % add 1st add'l cost layer L
%         as.plan(goal,3);    % setup costmap for specified goal
%         as.path(start);     % plan solution path start-goal, animate
%         P = as.path(start);    % plan solution path start-goal, return path
%     
% Notes::
% - Obstacles are represented by Inf in the costmap.
% 
% References::
% - A Pareto Optimal D* Search Algorithm for Multiobjective Path Planning, A. Lavin.
% - A Pareto Front-Based Multiobjective Path Planning Algorithm, A. Lavin.
% - Robotics, Vision & Control, Sec 5.2.2, Peter Corke, Springer, 2011.
%
% Author::
% Alexander Lavin
% 
% See also Navigation, Astar, AstarPO.

% Copyright (C) 1993-2015, by Peter I. Corke, Alexander Lavin
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

classdef AstarMOO < Navigation

    properties (SetAccess=private, GetAccess=private)

        % essential world info
        costmap   % world cost map: obstacle = Inf
        G         % index of goal point
        N         % number of objectives
        
        % info kept per cell (state):
        b            % backpointer (0 means not set)
        t            % tag: NEW OPEN CLOSED
        
        cost_g       % path distance summation
        cost_h       % path heuristic (state to goal) cost
        cost_01      % add'l cost layer 01 (unused)
        cost_02      % add'l cost layer 02 (unused)
        cost_03      % add'l cost layer 03 (unused)
                     % add more cost layers if needed...
        
        priority
        tie
        
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
        % AstarMOO class constructor:
        function as = AstarMOO(world, varargin)
            %AstarMOO.AstarMOO A*-MOO constructor
            %
            % AS = AstarMOO(MAP, OPTIONS) is a A* navigation object, and MAP is an
            % occupancy grid, a representation of a planar world as a
            % matrix whose elements are 0 (free space) or 1 (occupied).
            % The occupancy grid is coverted to a costmap with a unit cost
            % for traversing a cell.
            %
            % Options::
            % 'goal',G      Specify the goal point (2x1)
            % 'metric',M    Specify the distance metric as 'euclidean' (default)
            %               or 'cityblock'.
            % 'inflate',K   Inflate all obstacles by K cells.
            % 'quiet'       Don't display the progress spinner
            %
            % Other options are supported by the Navigation superclass.
            %
            % Notes::
            % - If MAP == 0 a random map is created.
            %
            % See also Navigation.Navigation.
            
            % invoke the superclass constructor
            as = as@Navigation(world, varargin{:}); % includes the occgrid

            % options
            opt.quiet = false;
            opt = tb_optparse(opt, varargin);
            as.quiet = opt.quiet;
            
            as.occgrid2costmap(as.occgrid);

            % initialize the A* state variables
            as.reset();
            if ~isempty(as.goal)
                as.goal_change();
            end
            as.changed = false;
        end
        
        function reset(as)
            %AstarMOO.reset Reset the planner
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
        
        %AstarMOO.goal_change Changes to costlayers due to new goal
        %position
        function goal_change(as)

            if isempty(as.b)
                return;
            end
            goal = as.goal;

            % keep goal in index rather than row,col format
            as.G = sub2ind(size(as.occgrid), goal(2), goal(1));
            as.INSERT(as.G, as.projectCost(as.G), 'goalset');
            as.cost_g(as.G) = 0;
            
            % new goal changes cost layers:
            as.calcHeuristic(as.occgrid, as.goal);
        end
        
        function s = char(as)
            %AstarMOO.char Convert navigation object to string
            %
            % AS.char() is a string representing the state of the Astar
            % object in human-readable form.
            %
            % See also AstarMOO.display, Navigation.char.
 
            % most of the work is done by the superclass
            s = char@Navigation(as);
        end

        function plot(as, varargin)
            %AstarMOO.plot Visualize navigation environment
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
        
        function plan(as, goal, N)
            %AstarMOO.plan Prep the grid for planning.
            %
            % AS.plan() updates AS with a costmap of distance to the
            % goal from every non-obstacle point in the map.  The goal is
            % as specified to the constructor.
            %
            % Inputs:
            %   goal: goal state coordinates
            %   N: number of optimization objectives; standard A* is 2
            %   (i.e. distance and heuristic)
            
            as.N = N; % number of optimization objectives
            as.openlist = zeros(as.N+1,0);
            
            % Setup cost layers. If a
            % cost layer is goal-dependent, it's setup function needs to
            % also be called in AS.goal_change(). If more cost layers are
            % needed, add similar to AS.cost_01.
            
            % initializations first:
            as.cost_g = zeros(size(as.occgrid));
            as.cost_h = zeros(size(as.occgrid)); % filled after setting goal below
            % if add'l costs haven't been added with addCost()
            if isempty(as.cost_01)
                as.cost_01 = zeros(size(as.occgrid));
            end
            if isempty(as.cost_02)
                as.cost_02 = zeros(size(as.occgrid));
            end
            if isempty(as.cost_03)
                as.cost_03 = zeros(size(as.occgrid));
            end
                        
            if nargin > 1
                as.goal = goal; % invokes superclass method set.goal()
            end
            if isempty(as.goal)
                error('must specify a goal point');
            end
            
            % Setup cost layers AS.cost_g and AS.cost_h.            
            % assign values to the distance cost layer, set as AS.costmap
            as.occgrid2costmap(as.occgrid);
            % assign values to the heuristic cost layer, set as AS.cost_h
            as.calcHeuristic(as.occgrid, as.goal);
            % Additional cost layers are added by the user with the
            % AS.addCost() method
            
            % Cost priority/tiebreaker: cost_g (distance to node)
            as.priority = as.cost_g;
            as.tie = 1; % first cost: cost_g
        end
        
        function P = path(as, start)
        %AstarMOO.path Find a path between two points
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
        % Calculate the solution path
        % Line comments LN reference A* pseudocode in Lavin's "A Pareto
        % Front-Based Multiobjective Path Planning Algorithm" where n is
        % the line number.
        function navigate_init(as, start)
            as.openlist = zeros(as.N+1,0); % make sure openlist is empty
            % begin with the start node
            start = sub2ind(size(as.costmap), start(2), start(1));
            as.openlist(1,1) = start;
            as.t(start) = as.OPEN;
            as.niter = 0; flag = 0; % init while loop
            % Plan the A* path
            while ~isempty(as.openlist) % L4
                % Normalize costs on the open list, choose expansion state:
                queue = normc(as.openlist(2:size(as.openlist,1),:)');
                [~,ind]=min(sum(queue,2));
                X = as.openlist(1,ind); % L5; minimum state on the open list
                as.DELETE(X); % L6; remove state from the openlist
                
                as.niter = as.niter + 1;
                
                if ~as.quiet && mod(as.niter, 20) == 0 % i.e. spinner every 20 iterations
                    as.spinner();
                end
                
                % Populate the openlist:
                for Y=as.neighbors(X) % L7,8; check node's 8 neighbors
                    if(Y==as.G) % L9; check for goal
                        as.b(Y) = X; 
                        as.updateCosts(Y,X,as.N)
                        flag = 1; % flag for goal
                        break;
                    end
                    if as.t(Y)==as.NEW && as.costmap(Y)~=Inf % hasn't been checked and isn't an obstacle
                        as.b(Y) = X;
                        as.updateCosts(Y,X,as.N); % as.N = 2 -> base case w/ only distance cost parameters (g and h)
                        % project node's costs into objective space:
                        objspace = as.projectCost(Y,X);
                        as.INSERT(Y, objspace, '');
                    end
                end                
                if as.verbose
                    disp(' ')
                end
                if flag==1 % goal found
                    break;
                end
            end
            if ~as.quiet
                fprintf('\r');
            end
            as.changed = false;
        end
        
        function layer = cost_get(as)
        %AstarMOO.cost_get Get the specified cost layer
            layer = as.cost_02;
        end
        
        function c = heuristic_get(as)
        %AstarMOO.heuristic_get Get the current heuristic map
        %
        % C = AS.heuristic_get() is the current heuristic map.  This map is the same size
        % as the occupancy grid and the value of each element is the shortest distance 
        % from the corresponding point in the map to the current goal.  It is computed
        % by Astar.plan.
        %
        % See also Astar.plan.
            c = as.cost_h;
        end

        function c = costmap_get(as)
        %AstarMOO.costmap_get Get the current costmap
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
        %AstarMOO.costmap_set Set the current costmap
        %
        % AS.costmap_set(C) sets the current costmap.  The cost map is the same size
        % as the occupancy grid and the value of each element represents the cost
        % of traversing the cell.  A high value indicates that the cell is more costly
        % (difficult) to traverese.  A value of Inf indicates an obstacle.
        %
        % Notes::
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
        %AstarMOO.costmap_modify Modify cost map
        %
        % AS.costmap_modify(P, NEW) modifies the cost map at P=[X,Y] to
        % have the value NEW.  If P (2xM) and NEW (1xM) then the cost of
        % the points defined by the columns of P are set to the corresponding
        % elements of NEW.
        %
        % Notes::
        % - After one or more point costs have been updated the path
        %   should be replanned by calling AS.plan().
        %
        % See also AstarMOO.costmap_set, AstarMOO.costmap_get.

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
                as.INSERT(X, as.h(X), 'modifycost');
            end
            as.changed = true;
        end 
        
        function addCost(as, layer, values)
        %AstarMOO.addCost Add an additional cost layer
        %
        % AS.addCost(LAYER, VALUES) adds the matrix specified by values as a
        % cost layer.  The layer number is given by LAYER, and VALUES has the same
        % size as the original occupancy grid.

            if size(values)~=size(as.occgrid)
                display('Layer size does not match the environment')
                return
            end
            if max(max(values))~=1 || min(min(values))~=0
                display('Layer values are not normalized [0:1]')
                return
            end
            
            if layer==1
                as.cost_01 = values;
            elseif layer==2
                as.cost_02 = values;
            elseif layer==3
                as.cost_03 = values;
            else
                display('Layer index out of range')
            end
            % If more cost layers needed, add additional elseif statements
            % as above.
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
 
             % NOTE: Only for costs that accumulate (i.e. sum) over the
            % path, and for dynamic costs.
            % E.g. the heuristic parameter AS.cost_h only needs updating
            % when the goal state changes; it's values are stored for each
            % cell.
            %
            % Location moving from state b to a.
            
        function k_new = updateCosts(as, a, b, obj)

            if nargout > 0
                k_new = as.cost_g(b) + as.dc(b,a);
                return
            end
            if obj == 0                        % just return what the new priority cost would be (k_new)
                return
            end
            if obj > 1                         % base case
                as.cost_g(a) = as.cost_g(b) + as.dc(b,a);
                % (no heuristic update needed)
            end
            if obj > 2                         % w/ cost_01: elevation
                % (no elevation update needed)
            end
            if obj > 3                         % w/ cost_02: solar
                sV = [cos(as.niter/100);sin(as.niter/100)]; % rotates 1rad per 100 steps
                as.cost_02(a) = dot(sV,as.vc(b,a));
            end
            if obj > 4                         % w/ cost_03: risk
                % (no risk update needed)
            end
        end
        
        % Returns the projection of state a into objective space. If
        % specified, location is moving from b to a.
        function pt = projectCost(as, a, b)
            switch nargin
                case 2
                    pt = [as.cost_g(a);
                          as.cost_h(a);
                          as.cost_01(a);
                          as.cost_02(a);
                          as.cost_03(a);
                          ];
                case 3
                    pt = [as.cost_g(b) + as.dc(a,b);
                          as.cost_h(a);
                          as.cost_01(a);
                          as.cost_02(a);
                          as.cost_03(a);
                          ];
                otherwise
                    return
            end
        end
        
        % X is new node with costs in array pt

        function INSERT(as, X, pt, where)

            if nargin>2
                as.message('insert (%s) %d = %f\n', where, X, pt);
            end
            
            i = find(as.openlist(1,:) == X);
            if length(i) > 1
                error('A*:INSERT: state in open list %d times', X);
            end

            if as.t(X) == as.NEW
                % add a new column to the open list
                as.openlist = [as.openlist [X; pt]];
            elseif as.t(X) == as.OPEN
                % L13: If a node with same position as successor is in the
                % OPEN list & has a lower f than successor, then skip this
                % successor.
                if pt(as.tie) < as.priority(X) % break tie with the sum of path costs
                    % add a new column to the open list
                    as.openlist = [as.openlist [X; pt]];
                end
            elseif as.t(X) == as.CLOSED
                % L14: If a node with same position as successor is in the
                % CLOSED list & has a lower f than successor, then skip this
                % successor.
                if pt(as.tie) < as.priority(X) % break tie with the sum of path costs
                    % add a new column to the open list
                    as.openlist = [as.openlist [X; pt]];
                end
            end

            % keep track of the max length of the openlist:
            if numcols(as.openlist) > as.openlist_maxlen
                as.openlist_maxlen = numcols(as.openlist);
            end

            as.t(X) = as.OPEN; % tag X as open
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
        
        % return the distance cost of moving from state X to state Y
        function cost = dc(as, X, Y)
            [r,c] = ind2sub(size(as.costmap), [X; Y]);
            dist = sqrt(sum(diff([r c]).^2));
            dcost = (as.costmap(X) + as.costmap(Y))/2;

            cost = dist * dcost;
        end

        % return the robot unit vector; direction of moving from state X to state Y
        function vector = vc(as, X, Y)
            [Xi,Xj] = ind2sub(size(as.occgrid),X);
            [Yi,Yj] = ind2sub(size(as.occgrid),Y);
            vector = [Yi-Xi;Yj-Xj];
            vector = vector/norm(vector);
%             slope = vector(2) / vector(1);
%             theta = dot([0,1],[vector])/(norm([0,1])*norm(vector));            
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
