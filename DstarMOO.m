%DstarMOO D*-MOO navigation class
%
% A concrete subclass of the Navigation class that implements the D*
% navigation algorithm; facilitates incremental replanning. This
% implementation of D* is intended for multiobjective optimization (MOO)
% problems - i.e. optimizes over several objectives/criteria.
%
% Methods::
% plan              Compute the cost map given a goal and map
% path              Compute a path to the goal
% visualize         Display the obstacle map (deprecated)
% plot              Display the obstacle map
% cost_get          Return the specified cost layer
% costmap_modify    Modify the costmap
% modify_cost       Modify the costmap (deprecated, use costmap_modify)
% costmap_get       Return the current costmap
% costmap_set       Set the current costmap
% distancemap_get   Set the current distance map
% display           Print the parameters in human readable form
% char              Convert to string
%
% Properties::
% TBD
%
% Example::
%        load map1              % load map
%        goal = [50,30];
%        start=[20,10];
%        ds = DstarMOO(map);    % create navigation object
%        ds.plan(goal,1)        % create plan for specified goal
%        ds.path(start)         % animate path from this start location
% Example 2:
%       goal = [100;100];
%       start = [1;1];
%       ds = DstarMOO(0);       % create Navigation object with random occupancy grid
%       ds.addCost(1,L);        % add 1st add'l cost layer L
%       ds.plan(goal,2);        % setup costmap for specified goal
%       ds.path(start);         % plan solution path start-goal, animate
%       P = as.path(start);     % plan solution path start-goal, return path
%
% Notes::
% - Obstacles are represented by Inf in the costmap.
%
% References::
% - The D* algorithm for real-time planning of optimal traverses,
%   A. Stentz, Tech. Rep. CMU-RI-TR-94-37, The Robotics Institute,
%   Carnegie-Mellon University, 1994.
% - A Pareto Optimal D* Search Algorithm for Multiobjective Path Planning,
%   A. Lavin.
% - Robotics, Vision & Control, Sec 5.2.2,
%   Peter Corke, Springer, 2011.
%
% Author::
% Alexander Lavin based on Dstar by Peter Corke
%
% See also Navigation, Dstar, DstarPO, Astar, DXform.

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
% All the state is kept in the structure called d
% X is an index into the array of states.
% state pointers are kept as matlab array index rather than row,col format

classdef DstarMOO < Navigation

    properties (SetAccess=private, GetAccess=private)

        costmap   % world cost map: obstacle = Inf
        G         % index of goal point
        N         % number of objectives

        % info kept per cell (state)
        b       % backpointer (0 means not set)
        t       % tag: NEW OPEN CLOSED
        
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

    methods

        % constructor
        function ds = DstarMOO(world, varargin)
            %DstarMOO.DstarMOO D*MOO constructor
            %
            % DS = DstarMOO(MAP, OPTIONS) is a D* navigation object, and MAP is an
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
            ds = ds@Navigation(world, varargin{:});

            % options
            opt.quiet = false;
            opt = tb_optparse(opt, varargin);
            ds.quiet = opt.quiet;

            ds.occgrid2costmap(ds.occgrid);

            % init the D* state variables
            ds.reset();
            if ~isempty(ds.goal)
                ds.goal_change();
            end
            ds.changed = false;
        end

        function reset(ds)
            %DstarMOO.reset Reset the planner
            %
            % DS.reset() resets the D* planner.  The next instantiation
            % of DS.plan() will perform a global replan.

            % build the matrices required to hold the state of each cell for D*
            ds.b = zeros(size(ds.costmap), 'uint32');     % backpointers
            ds.t = zeros(size(ds.costmap), 'uint8');      % tags
            ds.cost_g = Inf*ones(size(ds.costmap));       % path cost estimate
            ds.openlist = zeros(2,0);                     % the open list, one column per point

            ds.openlist_maxlen = -Inf;
        end

        function goal_change(ds)

            if isempty(ds.b)
                return;
            end
            goal = ds.goal;

            % keep goal in index rather than row,col format
            ds.G = sub2ind(size(ds.occgrid), goal(2), goal(1));
            ds.INSERT(ds.G, ds.projectCost(ds.G), 'goalset');
            ds.cost_g(ds.G) = 0;
            
            % new goal changes cost layers:
            ds.calcHeuristic(ds.occgrid, ds.goal);
        end

        function s = char(ds)
            %DstarMOO.char Convert navigation object to string
            %
            % DS.char() is a string representing the state of the Dstar
            % object in human-readable form.
            %
            % See also Dstar.display, Navigation.char.
 
            % most of the work is done by the superclass
            s = char@Navigation(ds);

            % Dstar specific stuff
            if ~isempty(ds.costmap)
                s = char(s, sprintf('  costmap: %dx%d, open list %d', size(ds.costmap), numcols(ds.openlist)));
            else
                s = char(s, sprintf('  costmap: empty:'));
            end
        end

        function plot(ds, varargin)
            %DstarMOO.plot Visualize navigation environment
            %
            % DS.plot() displays the occupancy grid and the goal distance
            % in a new figure.  The goal distance is shown by intensity which
            % increases with distance from the goal.  Obstacles are overlaid
            % and shown in red.
            %
            % DS.plot(P) as above but also overlays a path given by the set
            % of points P (Mx2).
            %
            % See also Navigation.plot.
            
            plot@Navigation(ds, 'distance', ds.cost_h, varargin{:});
        end

        % invoked by Navigation.step
        function n = next(ds, current)

            if ds.changed
                error('Cost map has changed, replan');
            end
            X = sub2ind(size(ds.costmap), current(2), current(1));
            X = ds.b(X);
            if X == 0
                n = [];
            else
                [r,c] = ind2sub(size(ds.costmap), X);
                n = [c;r];
            end
        end

        function plan(ds, goal, N)
            %DstarMOO.plan Plan path to goal
            %
            % DS.plan() updates DS with a costmap of distance to the
            % goal from every non-obstacle point in the map.  The goal is
            % as specified to the constructor.
            %
            % Note::
            % - If a path has already been planned, but the costmap was
            %   modified, then reinvoking this method will replan,
            %   incrementally updating the plan at lower cost than a full
            %   replan.
            %
            % Inputs:
            %   goal: goal state coordinates
            %   N: number of optimization objectives; standard D* is 2
            %   (i.e. distance and heuristic)
            
            ds.N = N; % number of optimization objectives
            ds.openlist = zeros(ds.N+1,0);
                      
            % Setup cost layers. If a
            % cost layer is goal-dependent, it's setup function needs to
            % also be called in DS.goal_change(). If more cost layers are
            % needed, add similar to DS.cost_01.
            
            % initializations first:
            ds.cost_g = zeros(size(ds.occgrid));
            ds.cost_h = zeros(size(ds.occgrid)); % filled after setting goal below
            % if add'l costs haven't been added with addCost()
            if isempty(ds.cost_01)
                ds.cost_01 = zeros(size(ds.occgrid));
            end
            if isempty(ds.cost_02)
                ds.cost_02 = zeros(size(ds.occgrid));
            end
            if isempty(ds.cost_03)
                ds.cost_03 = zeros(size(ds.occgrid));
            end
            
            if nargin > 1
                ds.goal = goal; % invokes superclass method set.goal()
            end
            % for replanning no goal is needed, 
            if isempty(ds.goal)
                error('must specify a goal point');
            end

            % Setup cost layers DS.cost_g and DS.cost_h.
            % assign values to the distance cost layer, set as DS.costmap
            ds.occgrid2costmap(ds.occgrid);
            % assign values to the heuristic cost layer, set as DS.cost_h
            ds.calcHeuristic(ds.occgrid, ds.goal);
            % Additional cost layers are added by the user with the
            % DS.addCost() method
            
            % Cost priority/tiebreaker: cost_g (distance to node)
            ds.priority = ds.cost_g;
            ds.tie = 1; % first cost: cost_g
            
            ds.niter = 0;
            while true
                if ~ds.quiet && mod(ds.niter, 20) == 0
                    ds.spinner();
                end
                ds.niter = ds.niter + 1;

                if ds.PROCESS_STATE() < 0
                    break;
                end
                if ds.verbose
                    disp(' ')
                end
            end
            if ~ds.quiet
                fprintf('\r');
            end
            ds.changed = false;
        end
        
        function layer = cost_get(ds)
        %DstarMOO.cost_get Get the specified cost layer
            layer = ds.cost_02;
        end

        function c = distancemap_get(ds)
        %DstarMOO.distancemap_get Get the current distance map
        %
        % C = DS.distancemap_get() is the current distance map.  This map is the same size
        % as the occupancy grid and the value of each element is the shortest distance 
        % from the corresponding point in the map to the current goal.  It is computed
        % by Dstar.plan.
        %
        % See also Dstar.plan.
            c = ds.cost_h;
        end

        function c = costmap_get(ds)
        %DstarMOO.costmap_get Get the current costmap
        %
        % C = DS.costmap_get() is the current costmap.  The cost map is the same size
        % as the occupancy grid and the value of each element represents the cost
        % of traversing the cell.  It is autogenerated by the class constructor from
        % the occupancy grid such that:
        % - free cell (occupancy 0) has a cost of 1
        % - occupied cell (occupancy >0) has a cost of Inf
        %
        % See also Dstar.costmap_set, Dstar.costmap_modify.

            c = ds.costmap;
        end

        function costmap_set(ds, costmap)
        %DstarMOO.costmap_set Set the current costmap
        %
        % DS.costmap_set(C) sets the current costmap.  The cost map is the same size
        % as the occupancy grid and the value of each element represents the cost
        % of traversing the cell.  A high value indicates that the cell is more costly
        % (difficult) to traverese.  A value of Inf indicates an obstacle.
        %
        % Notes::
        % - After the cost map is changed the path should be replanned by 
        %   calling DS.plan(). 
        %
        % See also Dstar.costmap_get, Dstar.costmap_modify.
            if ~all(size(costmap) == size(ds.occgrid))
                error('costmap must be same size as occupancy grid');
            end
            ds.costmap = costmap;
            ds.changed = true;
        end

        function costmap_modify(ds, point, newcost)
        %DstarMOO.costmap_modify Modify cost map
        %
        % DS.costmap_modify(P, NEW) modifies the cost map at P=[X,Y] to
        % have the value NEW.  If P (2xM) and NEW (1xM) then the cost of
        % the points defined by the columns of P are set to the corresponding
        % elements of NEW.
        %
        % Notes::
        % - After one or more point costs have been updated the path
        %   should be replanned by calling DS.plan(). 
        % - Replaces modify_cost, same syntax.
        %
        % See also Dstar.costmap_set, Dstar.costmap_get.

            if numel(point) == 2
                % for case of single point ensure it is a column vector
                point = point(:);
            end
            if numcols(point) ~= numcols(newcost)
                error('number of columns in point must match columns in newcost');
            end
            for i=1:numcols(point)
                X = sub2ind(size(ds.costmap), point(2,i), point(1,i));
                ds.costmap(X) = newcost(i);
            end
            if ds.t(X) == ds.CLOSED
                ds.INSERT(X, ds.h(X), 'modifycost');
            end
            ds.changed = true;
        end
        
        function addCost(ds, layer, values)
        %DstarMOO.addCost Add an additional cost layer
        %
        % DS.addCost(layer,values) adds the matrix specified by values as a
        % cost layer.
        % Inputs
        %   layer: 1, 2, or 3 to specify which cost layer to add
        %   values: normalized matrix the size of the environment (100x100)
            if size(values)~=size(ds.occgrid)
                display('Error: layer size does not match the environment')
                return
            end
            if max(max(values))~=1 || min(min(values))~=0
                display('Warning: layer values are not normalized [0:1]')
            end
            
            if layer==1
                ds.cost_01 = values;
            elseif layer==2
                ds.cost_02 = values;
            elseif layer==3
                ds.cost_03 = values;
            else
                display('Layer index out of range')
            end
            % If more cost layers needed, add additional elseif statements
            % as above.
        end
        
    end % public methods

    methods (Access=protected)

        function occgrid2costmap(ds, og, cost)
            if nargin < 3
                cost = 1;
            end
            ds.costmap = og;
            ds.costmap(ds.costmap==1) = Inf;      % occupied cells have Inf driving cost
            ds.costmap(ds.costmap==0) = cost;     % unoccupied cells have driving cost
        end

        function calcHeuristic(ds, grid, goal)
            ds.cost_h=zeros(size(grid));
            for ii=1:size(grid,1)
                for jj=1:size(grid,2)
                    ds.cost_h(ii,jj)=sqrt((ii-goal(1))^2+(jj-goal(2))^2);
                end
            end
        end
        
        % The main D* function as per the Stentz paper, revised for MOO
        % path planning per the Lavin paper. Comments Ln are the original
        % line numbers.
        function r = PROCESS_STATE(d)
            % States with the lowest k value are removed from the
            % open list
            queue = normc(d.openlist(2:size(d.openlist,1),:)');
            [~,ind]=min(sum(queue,2));
            X = d.openlist(1,ind); % L1
            
            if isempty(X) % L2
                r = -1;
                return;
            end

            k_old = d.GET_KMIN(); d.DELETE(X); % L3
            
            d.priority = d.cost_g; % updates priority cost layer

            if k_old < d.priority(X) % L4
                d.message('k_old < h(X):  %f %f\n', k_old, d.priority(X));
                for Y=d.neighbours(X) % L5
                    if (d.priority(Y) <= k_old) && (d.priority(X) > d.updateCosts(Y,X,0))  % L6
                        d.b(X) = Y;
                        d.updateCosts(X,Y,d.N); % L7
                    end
                end
            end

            % can we lower the path cost of any neighbours?
            if k_old == d.priority(X) % L8
                d.message('k_old == h(X): %f\n', k_old);
                for Y=d.neighbours(X) % L9
                    if (d.t(Y) == d.NEW) || ... % L10-12
                            ( (d.b(Y) == X) && (d.priority(Y) ~= d.updateCosts(Y,X,0)) ) || ...
                            ( (d.b(Y) ~= X) && (d.priority(Y) > d.updateCosts(Y,X,0)) )
                        % Update and project the costs:
                        d.updateCosts(Y,X,d.N);
                        objspace = d.projectCost(Y,X);
                        d.b(Y) = X;
                        d.INSERT(Y, objspace, 'L13'); % L13
                    end
                 end
            else % L14
                d.message('k_old > h(X)');
                for Y=d.neighbours(X) % L15
                    if (d.t(Y) == d.NEW) || ( (d.b(Y) == X) && (d.priority(Y) ~= d.updateCosts(Y,X,0)) )
                        d.updateCosts(Y,X,d.N);
                        objspace = d.projectCost(Y,X);
                        d.b(Y) = X;
                        d.INSERT(Y, objspace, 'L18'); % L18
                    else
                        if ( (d.b(Y) ~= X) && (d.priority(Y) > d.updateCosts(Y,X,0)) )
                            d.INSERT(X, d.projectCost(X), 'L21'); % L21
                        else
                            if (d.b(Y) ~= X) && (d.priority(X) > d.updateCosts(Y,X,0)) && ...
                                    (d.t(Y) == d.CLOSED) && d.priority(Y) > k_old
                                d.INSERT(Y, d.projectCost(Y), 'L25'); % L25
                            end
                        end
                    end
                 end
            end
            
            r = 0;
            return;
        end % process_state
        
        function k_new = updateCosts(ds, a, b, obj)
            % NOTE: Only for costs that accumulate (i.e. sum) over the
            % path, and for dynamic costs.
            % E.g. the heuristic parameter DS.cost_h only needs updating
            % when the goal state changes; it's values are stored for each
            % cell.
            %
            % Location moving from state b to a.
            if nargout > 0
                k_new = ds.cost_g(b) + ds.dc(b,a);
                return
            end
            if obj == 0                        % just return what the new priority cost would be (k_new)
                return
            end
            if obj > 1                         % base case
                ds.cost_g(a) = ds.cost_g(b) + ds.dc(b,a);
                % (no heuristic update needed)
            end
            if obj > 2                         % w/ cost_01: elevation
                % (no elevation update needed)
            end
            if obj > 3                         % w/ cost_02: solar
                sV = [cos(ds.niter/100);sin(ds.niter/100)]; % rotates 1rad per 100 steps
                ds.cost_02(a) = dot(sV,ds.vc(b,a));
            end
            if obj > 4                         % w/ cost_03: risk
                % (no risk update needed)
            end
        end
        
        function pt = projectCost(ds, a, b)
            % Returns the projection of state a into objective space. If
            % specified, location is moving from b to a.
            switch nargin
                case 2
                    pt = [ds.cost_g(a);
                          ds.cost_h(a);
                          ds.cost_01(a);
                          ds.cost_02(a);
                          ds.cost_03(a);
                          ];
                case 3
                    pt = [ds.cost_g(b) + ds.dc(a,b);
                          ds.cost_h(a);
                          ds.cost_01(a);
                          ds.cost_02(a);
                          ds.cost_03(a);
                          ];
                otherwise
                    return
            end
        end

        function INSERT(ds, X, pt, where)
            % Add state X to the openlist with objective space values
            % specified by pt.
            
            % where is for diagnostic purposes only
            ds.message('insert (%s) %d = %f\n', where, X, pt);

            i = find(ds.openlist(1,:) == X);
            if length(i) > 1
                error('D*:INSERT: state in open list %d times', X);
            end

            if ds.t(X) == ds.NEW
                % add a new column to the open list
                ds.openlist = [ds.openlist [X; pt]];
            elseif ds.t(X) == ds.OPEN
%                 k_new = min( ds.openlist(2,i), h_new );
            elseif ds.t(X) == ds.CLOSED
                if pt(ds.tie) < ds.priority(X) % break tie with the sum of path costs
                    % add a new column to the open list
                    ds.openlist = [ds.openlist [X; pt]];
                end
            end

            % keep track of the max length of the openlist:
            if numcols(ds.openlist) > ds.openlist_maxlen
                ds.openlist_maxlen = numcols(ds.openlist);
            end

            ds.t(X) = ds.OPEN;
        end

        function DELETE(ds, X)
            ds.message('delete %d\n', X);
            i = find(ds.openlist(1,:) == X);
            if length(i) ~= 1
                error('D*:DELETE: state %d doesnt exist', X);
            end
            ds.openlist(:,i) = []; % remove the column
            ds.t(X) = ds.CLOSED;
        end

        function kmin = GET_KMIN(ds)
            kmin = min(ds.openlist(2,:));
        end

        % return the cost of moving from state X to state Y
        function cost = dc(ds, X, Y)
            [r,c] = ind2sub(size(ds.costmap), [X; Y]);
            dist = sqrt(sum(diff([r c]).^2));
            dcost = (ds.costmap(X) + ds.costmap(Y))/2;

            cost = dist * dcost;
        end

        % return the robot unit vector; direction of moving from state X to state Y
        function vector = vc(ds, X, Y)
            [Xi,Xj] = ind2sub(size(ds.occgrid),X);
            [Yi,Yj] = ind2sub(size(ds.occgrid),Y);
            vector = [Yi-Xi;Yj-Xj];
            vector = vector/norm(vector);
%             slope = vector(2) / vector(1);
%             theta = dot([0,1],[vector])/(norm([0,1])*norm(vector));            
        end
        
        % return index of neighbour states as a row vector
        function Y = neighbours(ds, X)
            dims = size(ds.costmap);
            [r,c] = ind2sub(dims, X);

            % list of 8-way neighbours
            Y = [r-1 r-1 r-1 r r  r+1 r+1 r+1; c-1 c c+1 c-1 c+1 c-1 c c+1];
            k = (min(Y)>0) & (Y(1,:)<=dims(1)) & (Y(2,:)<=dims(2));
            Y = Y(:,k);
            Y = sub2ind(dims, Y(1,:)', Y(2,:)')';
        end

    end % protected methods
end % classdef
