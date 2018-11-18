% Astar (A*)
% A* navigation class
% 
% A concrete subclass of the Navigation class that implements the A*
% navigation algorithm. Methods included are for the standard case,
% multiobjective optimization (MOO) -- i.e. optimizes over several 
% objectives/criteria -- and the A*-PO algorithms for MOO that utilizes
% Pareto optimality.
% 
% Methods:
% 	plan            Compute the cost map given a goal and map
% 	path            Compute a path to the goal
% 	visualize       Display the obstacle map (deprecated)
% 	plot            Display the obstacle map
% 	costmap_modify 	Modify the costmap
% 	costmap_get     Return the current costmap
% 	costmap_set     Set the current costmap
% 	display         Print the parameters in human readable form
% 	char            Convert to string
% 
% Properties:
% TBD
% 
% Example 1::
%        load map1              % load map
%        goal = [50;30];
%        start=[20;10];
%        as = Astar(map);       % create Navigation object
%        as.plan(goal,2,3,0);   % setup costmap for specified goal; 
%                               % standard D* algorithm w/ 2 objectives
%                               % and 3 costmap layers
%        as.path(start);		% plan solution path start-to-goal, animate
%        P = as.path(start);	% plan solution path start-to-goal, return 
%                               % path
% Example 2::
%       goal = [100;100];
%       start = [1;1];
%       as = Astar(0);          % create Navigation object with pseudo-
%                               % random occupancy grid
%       ds.addCost(terrain);    % terrain is a 100x100 matrix of 
%                               % elevations [0,1]
% 	    ds.plan(goal,3,4,0);    % setup costmap for specified goal
%                               % (3 and 4 include the added terrain cost)
%       as.path(start);         % plan solution path start-goal, animate
%       P = as.path(start);     % plan solution path start-goal, return 
%                               % path
%     
% Notes
% - Obstacles are represented by Inf in the costmap.
% 
% References
% - A Pareto Optimal D* Search Algorithm for Multiobjective Path Planning,
%   A. Lavin.
% - A Pareto Front-Based Multiobjective Path Planning Algorithm, A. Lavin.
% - Robotics, Vision & Control, Sec 5.2.2, Peter Corke, Springer, 2011.
% 
% See Also Navigation, Dstar


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


% Implementation notes:
%
% X is an index into the array of states.
% State pointers are kept as matlab array index rather than row,col format.

classdef Astar < Navigation

    properties (SetAccess=private, GetAccess=private)

        % essential world info
        costmap   % cost layers (1st is map)
        G         % index of goal point
        N         % number of objectives
        L         % number of cost layers
        
        % info kept per cell (state):
        b         % backpointer (0 means not set)
        t         % tag: NEW/OPEN/CLOSED
        
        algorithm % A*, A*-MOO, or A*-PO
        tie
        openlist  % priority queue with states and their costs
        niter
        changed
        openlist_maxlen
        quiet     % specifies verbosity

        % tag state values
        NEW = 0;
        OPEN = 1;
        CLOSED = 2;
    end
    
    
    methods  % start of public methods

        function as = Astar(world, varargin)
            %Astar.Astar A* constructor
            %
            % AS = Astar(MAP, OPTIONS) is a A* navigation object, and MAP 
            % is an occupancy grid, a representation of a planar world as 
            % a matrix whose elements are 0 (free space) or 1 (occupied).
            % The occupancy grid is coverted to a costmap with a unit cost
            % for traversing a cell.
            %
            % Options::
            % 'world' = 0   will call for a pseudo-random occupancy grid
            % 'goal',G      Specify the goal point (2x1)
            % 'metric',M    Specify the distance metric as 'Euclidean'
            %               (default) or 'cityblock'
            % 'inflate',K   Inflate all obstacles by K cells
            % 'quiet'       Don't display the progress spinner
            %
            % Other options are supported by the Navigation superclass.
            %
            % See also Navigation.Navigation.
            
            % Invoke the superclass constructor
            as = as@Navigation(world, varargin{:});  % includes the occgrid

            % Options
            opt.quiet = false;
            opt = tb_optparse(opt, varargin);
            as.quiet = opt.quiet;
            
            as.occgrid2costmap(as.occgrid);

            % Initialize A* state variables
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

            % Build the matrices required to hold the state of each cell
            as.b = zeros(size(as.occgrid), 'uint32');     % backpointers
            as.t = zeros(size(as.occgrid), 'uint8');      % tags, all NEW=0
            as.costmap(:,:,2) = zeros(size(as.occgrid));  % path cost g
            as.costmap(:,:,3) = zeros(size(as.occgrid));  % path cost h
            
            % Priority queue has col for each open state, one row for the
            % state location, and rows for each cost layer
            as.openlist = zeros(as.L+1,0);    
            as.openlist_maxlen = -Inf;
        end
        
        
        function goal_change(as)
            %Astar.goal_change Changes the costlayers due to new goal
            %position
            if isempty(as.b)
                return;
            end
            goal = as.goal;

            % Keep goal in index rather than row,col format
            as.G = sub2ind(size(as.occgrid), goal(2), goal(1));
            as.INSERT(as.G, as.projectCost(as.G), 'goalset');
            as.costmap(goal(2),goal(1),2) = 0;
            
            % If new goal modifies costs for a layer, recalculate here
            as.calcHeuristic(as.occgrid, as.goal);
        end
        
        
        function s = char(as)
            %Astar.char Convert Navigation object to string
            %
            % AS.char() is a string representing the state of the Astar
            % object in human-readable form.
            %
            % See also Astar.display, Navigation.char.
 
            % Work is done by the superclass
            s = char@Navigation(as);
        end

        
        function plot(as, varargin)
            %Astar.plot Visualize navigation environment
            %
            % AS.plot() displays the occupancy grid and the goal distance
            % in a new figure.  The goal distance is shown by intensity 
            % which increases with distance from the goal.  Obstacles are 
            % overlaid and shown in red.
            %
            % AS.plot(P) as above but also overlays a path given by the set
            % of points P (Mx2).
            %
            % See also Navigation.plot.
            
            plot@Navigation(as, 'distance', as.costmap(:,:,3), varargin{:});
        end

        
        function n = next(as, current)
            % Invoked by Navigation.step
            % Backpropagate from goal to start
            % Return [col;row] of previous step
            if as.changed
                error('Cost map has changed, replan');
            end
            X = sub2ind(size(as.occgrid), current(2), current(1));
            % Set X as the backpointer of X
            X = as.b(X);
            if X == 0
                % Goal (no further backpointer)
                n = [];
            else
                [r,c] = ind2sub(size(as.occgrid), X);
                n = [c;r];
            end
        end        
        
        
        function plan(as, goal, N, layers, algorithm)
            %Astar.plan Prep the grid for planning.
            %
            % AS.plan() updates AS with a costmap of distance to the
            % goal from every non-obstacle point in the map.  The goal is
            % as specified to the constructor.
            %
            % Inputs:
            %   goal: goal state coordinates
            %   N: number of optimization objectives; standard A* is 2
            %   (i.e. distance and heuristic)
            %   layers: number of cost layers in costmap
            %   algorithm: specify standard A*(0), A*-MOO (1), A*-PO (2)
                        
            % Setup parameters
            if nargin < 3
                N = 2;
            end
            if nargin < 4
                layers = 3;
            end
            if nargin < 5
                algorithm = 0;
            end
            as.N = N;
            as.L = layers;
            as.algorithm = algorithm;
            as.openlist = zeros(as.L+1,0);
            
            % Initialize cost layers
            for a = 2:as.L
                as.costmap(:,:,a) = zeros(size(as.occgrid));
            end
            
            % Cost priority/tiebreaker: layer 2 (distance to node)
            as.tie = 2;
            
            % Set goal
            if nargin > 1
                as.goal = goal;  % invokes superclass method set.goal()
            end
            if isempty(as.goal)
                error('must specify a goal point');
            end
            
            % Populate heuristic cost layer
            as.calcHeuristic(as.occgrid, as.goal);
        end
        
        
        function P = path(as, start)
        %Astar.path Find a path between two points
        %
        % AS.path(START) finds and displays a path from START to GOAL
        % which is overlaid on the occupancy grid.
        %
        % P = AS.path(START) returns the path (2xM) from START to GOAL.
            if nargin < 1
                error('must specify start state');
            end

            % Invoke the superclass path function, which iterates on our
            % next method
%             start = [start; 1];  % specifies backpropogation for NAV.path()
%             temp = start;
%             start = as.goal;
%             as.goal = temp;
            
            if nargout == 0
                path@Navigation(as, start);
            else
                P = path@Navigation(as, start);
            end
        end
        
        
        % Handler invoked by Navigation.path() to start the navigation
        % process -- calculate the solution path.
        % Line comments Ln reference A* pseudocode in Lavin's "A Pareto
        % Front-Based Multiobjective Path Planning Algorithm" where n is
        % the line number.
        function navigate_init(as, start)
            as.openlist = zeros(as.L+1,0);  % openlist must be empty
            % Begin search with the start node
            start = sub2ind(size(as.occgrid), start(2), start(1));
            as.openlist(1,1) = start;
            as.t(start) = as.OPEN;

            % Plan the A* path
            as.niter = 0; flag = 0;
            while ~isempty(as.openlist)                              % L4
                % Normalize costs on the open list, choose expansion state
                queue = normc(as.openlist(2:size(as.openlist,1),:)');
                if as.algorithm == 2
                    % Get Pareto optimal point off the open list
                    front = as.openlist(:,paretofront(queue));
                    [~,col] = min(front(as.tie+1,:));
                    X = front(1,col);                                % L5
                else
                    [~,ind]=min(sum(queue,2));                       
                    X = as.openlist(1,ind);                          % L5
                end
                    as.DELETE(X);                                    % L6
                
                as.niter = as.niter + 1;
                if ~as.quiet && mod(as.niter, 20) == 0
                    as.spinner();
                end
                
                % Populate the openlist
                for Y=as.neighbors(X)                                % L7,8
                    if(Y==as.G)                                      % L9
                        as.b(Y) = X; 
                        as.updateCosts(Y,X,as.N)
                        flag = 1;  % flag for goal
                        break;
                    end
                    if as.t(Y)==as.NEW && as.costmap(Y)~=Inf
                        as.b(Y) = X;
                        as.updateCosts(Y,X,as.N);
                        % Project node's costs into objective space:
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
        
        
        function layer = cost_get(as, layer)
        %Astar.cost_get Get the specified cost layer
            layer = as.costmap(:,:,layer);
        end
        
        
        function c = heurstic_get(as)
        %Astar.heuristice_get Get the current heuristic map
        %
        % C = AS.heuristice_get() is the current heuristic layer. It is
        % computed in Astar.plan.
        %
        % See also Astar.plan.
            c = as.costmap(:,:,3);
        end

        
        function c = costmap_get(as)
        %Astar.costmap_get Get the current costmap
        %
        % C = AS.costmap_get() is the current costmap.
        % The value of each element represents the cost of traversing the 
        % cell.  It is autogenerated by the class constructor from the
        % occupancy grid such that:
        % - free cell (occupancy 0) has a cost of 1
        % - occupied cell (occupancy >0) has a cost of Inf
        %
        % See also Astar.costmap_set, Astar.costmap_modify.
            c = as.costmap;
        end
        
        
        function costmap_set(as, costmap)
        %Astar.costmap_set Set the current costmap
        %
        % AS.costmap_set(C) sets the current costmap.
        % This method accepts the full costmap -- i.e. all layers.
        %
        % Notes:
        % - After the cost map is changed the path should be replanned by 
        %   calling AS.plan(). 
        %
        % See also Astar.costmap_get, Astar.costmap_modify.
            [i,j,k] = size(costmap);
            if ~all([i,j] == size(as.occgrid))
                error('costmap must be same size as occupancy grid');
            end
            as.L = k;  % set the number of cost layers
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
        %   should be replanned by calling AS.plan().
        %
        % See also Astar.costmap_set, Astar.costmap_get.
            if (newcost < 0) || (1 < newcost)
                error('new cost value must be normlaized [0,1]')
            end
            
            [i,j,k] = size(as.costmap);
            if (point(1) < 0) || (point(1) > i)
                error('1st dimension of point is out of bounds')
            end
            if (point(2) < 0) || (point(2) > j)
                error('2nd dimension of point is out of bounds')
            end
            if (point(2) < 0) || (point(3) > k)
                error('3rd dimension of point is out of bounds')
            end

            as.costmap(point) = newcost;
        end 
        
        
        function addCost(as, values)
        %Astar.addCost Add an additional cost layer
        %
        % AS.addCost(values) adds the matrix specified by values as a
        % cost layer.
        % Inputs
        %   values: normalized matrix the size of the environment
            [i,j,k] = size(as.costmap);
            
            if [i,j]~=size(as.occgrid)
                error('layer size does not match the environment')
            end
            if max(max(values))~=1 || min(min(values))~=0
                error('layer values are not normalized [0,1]')
            end
            
            as.costmap(:,:,k+1) = values;
        end
        
        
        function flag = backProp(as)
            flag = 1;
        end
        
    end  % end of public methods
    
    
    methods (Access=protected)  % start of private methods
        
        function occgrid2costmap(as, og, cost)
            if nargin < 3
                cost = 1;
            end
            og(og==1) = Inf;  % occupied cells -> infinite path cost
            og(og==0) = cost;  % unoccupied cells -> path cost
            as.costmap(:,:,1) = og;
        end
        
        
        function calcHeuristic(as, grid, goal)
            as.costmap(:,:,3) = zeros(size(grid));
            for ii=1:size(grid,1)
                for jj=1:size(grid,2)
                    as.costmap(ii,jj,3) = sqrt((ii-goal(1))^2+(jj-goal(2))^2);
                end
            end
        end
        
        function k_new = updateCosts(as, a, b, obj)
            % NOTE: Only for costs that accumulate (i.e. sum) over the
            % path, and for dynamic costs.
            % E.g. the heuristic parameter only needs updating when the
            % goal state changes; its values are stored for each cell.
            %
            % Location moving from state b to a.
            %
            % The costs are coded to be (1) distance, (2) heuristic, (3)
            % elevation, (4) solar deviation, and (5) risk. If deviating
            % from these costs (in this order) you MUST EDIT THIS METHOD.
            [i,j,~] = size(as.costmap);
            
            if nargout > 0
                k_new = as.costmap(i*j+b) + as.dc(b,a);
                return
            end
            if obj == 0
                % Return what the new priority cost would be (k_new)
                return
            end
            if obj > 1
                % Standard A* search
                as.costmap(i*j+a) = as.costmap(i*j+b) + as.dc(b,a);
                % (no heuristic update needed)
            end
            if obj > 2
                % W/ elevation costs
                % (no elevation update needed)
            end
            if obj > 3
                % W/ solar costs
                % Rotate the solar vector 1rad per 100 steps
                sV = [cos(as.niter/100);sin(as.niter/100)];
                as.costmap(4*i*j+a) = dot(sV,as.vc(b,a));
            end
            if obj > 4
                % W/ risk costs
                % (no risk update needed)
            end
        end
        
        
        function pt = projectCost(as, a, b)
            % Returns the projection of state a into objective space. If
            % specified, location is moving from b to a (case 3).
            [i,j,k] = size(as.costmap);
            pt(1) = as.costmap(a);
            switch nargin
                case 2
                    pt(2) = as.costmap(i*j+a);
                case 3
                    pt(2) = as.costmap(i*j+b) + as.dc(a,b);
                otherwise
                    return
            end
            for n=3:k
                pt(n) = as.costmap((n-1)*i*j+a);
            end
        end
        
        
        function INSERT(as, X, pt, where)
            % Add state X to the openlist with objective space values
            % specified by pt.

            if nargin>2
                as.message('insert (%s) %d = %f\n', where, X, pt);
            end
            
            i = find(as.openlist(1,:) == X);
            if length(i) > 1
                error('A*:INSERT: state in open list %d times', X);
            end

            [i,j,~] = size(as.costmap);
            if (as.t(X) == as.OPEN || as.CLOSED) && ...
               (pt(as.tie) > as.costmap((as.tie-1)*i*j+X))
                % L13/14: If a node with same position as successor is in 
                % the OPEN/CLOSED list & has a lower f than successor, 
                % then skip this successor.
            else
                % Add a new column to the open list for this node
                as.openlist = [as.openlist [X; pt(:)]];
            end
            
            % Keep track of the max length of the openlist
            if numcols(as.openlist) > as.openlist_maxlen
                as.openlist_maxlen = numcols(as.openlist);
            end

            % Tag state X as open
            as.t(X) = as.OPEN;
        end

        
        function DELETE(as, X)
            as.message('delete %d\n', X);
            i = find(as.openlist(1,:) == X);
            if length(i) ~= 1
                error('A*:DELETE: state %d does not exist', X);
            end
            
            % Remove the column, close the state
            as.openlist(:,i) = [];
            as.t(X) = as.CLOSED;
        end
        

        function cost = dc(as, X, Y)
            % Return the distance cost of moving from state X to state Y
            [r,c] = ind2sub(size(as.occgrid), [X; Y]);
            dist = sqrt(sum(diff([r c]).^2));
            dcost = (as.costmap(X) + as.costmap(Y))/2;

            cost = dist * dcost;
        end
        

        function vector = vc(as, X, Y)
            % Return the robot unit vector -- direction of moving from 
            % state X to state Y
            [Xi,Xj] = ind2sub(size(as.occgrid),X);
            [Yi,Yj] = ind2sub(size(as.occgrid),Y);
            vector = [Yi-Xi;Yj-Xj];
            vector = vector/norm(vector);           
        end
        
        
        function Y = neighbors(as, X)
            % Return indices of neighbor states (max 8) as a row vector
            dims = size(as.occgrid);
            [r,c] = ind2sub(dims, X);

            % Of 8-way neighbors, only use those w/in grid bounds
            Y = [r-1 r-1 r-1 r r  r+1 r+1 r+1; c-1 c c+1 c-1 c+1 c-1 c c+1];
            k = (min(Y)>0) & (Y(1,:)<=dims(1)) & (Y(2,:)<=dims(2));
            Y = Y(:,k);
            Y = sub2ind(dims, Y(1,:)', Y(2,:)')';
        end
 
    end  % end of private methods
    
end
