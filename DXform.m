%DXform Distance transform navigation class
%
% A concrete subclass of Navigation that implements the distance transform
% navigation algorithm.  This provides minimum distance paths.
%
% Methods::
%
% plan         Compute the cost map given a goal and map
% path         Compute a path to the goal
% visualize    Display the obstacle map
% display      Print the parameters in human readable form
% char         Convert the parameters to a human readable string
%
% Properties::
%
% metric     The distance metric, can be 'euclidean' (default) or 'cityblock'
% distance   The distance transform of the occupancy grid
%
% Example::
%
%    load map1
%    dx = DXform(map);
%    dx.plan(goal)
%    dx.path(start)
%
% See also Navigation, Dstar, PRM, distancexform.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

classdef DXform < Navigation

    properties
        metric;     % distance metric
        distance;   % distance transform results
    end

    methods

        function dx = DXform(world, varargin)
            %DXform.DXform Distance transform navigation constructor
            %
            % DX = DXform(MAP) is a distance transform navigation object,
            % and MAP is an occupancy grid, a representation of a planar
            % world as a matrix whose elements are 0 (free space) or 1
            % (occupied).
            %
            % DS = Dstar(MAP, GOAL) as above but specify the goal point.
            %
            % See also Navigation.Navigation.

            % TODO NEEDS PROPER ARG HANDLER


            % invoke the superclass constructor
            dx = dx@Navigation(world, varargin{:});

            dx.metric = 'euclidean';

            %% MORPH THE WORLD

            % set up the distance metrics
            if nargin < 3
                %dx.metric = 'cityblock';
            end

        end

        function s = char(dx)
        %DXform.char Convert navigation object to string
        %
        % DX.char() is a string representing the state of the navigation
        % object in human-readable form.
        %
        % See also DXform.display.
 
            s = sprintf('dx2 object:');
            if ~isempty(dx.goal)
                s = strcat( sprintf(' goal=%d,%d\n', dx.goal(1), dx.goal(2)) );
            end
        end

        function setworld()

            % invoked by superclass constructor
        end

        function setgoal(dx, goal)
            % for the imorph primitive we need to set the target pixel to 0,
            % obstacles to NaN and the rest to Inf.
            % invoked by superclass constructor

            if ~isempty(goal)
                % point goal case
                world(world>0) = NaN;
                world(world==0) = Inf;
                if world(goal(2), goal(1)) > 0
                    error('goal inside obstacle')
                else
                    world(goal(2), goal(1)) = 0;
                end
            else
                world(world==0) = Inf;
                world(world~=Inf) = 0;
            end

        end
            
        function plan(dx, goal, show)
            %DXform.plan Plan path to goal
            %
            % DX.plan() updates DX with a costmap of distance to the
            % goal from every non-obstacle point in the map.  The goal is
            % as specified to the constructor.
            %
            % DX.plan(GOAL) as above but uses the specified goal
            %
            % DX.plan(GOAL, S) as above but displays the evolution of the
            % costmap, with one iteration displayed every S seconds.

            %
            % plan()
            % plan(dt)
            % plan(goal)
            % plan(goal, dt)
            %
            %DISTANCEXFORM Distance transform of occupancy grid
            %
            %   dist = distancexform(world, goal)
            %
            %   Compute the distance transform for the occupancy grid, world, with
            %  respect to the specified goal point (x,y).
            %
            %   dist = distancexform(world, goal, metric)
            %
            %  Specify the metric, either 'cityblock' or 'Euclidean'
            %
            %   dist = distancexform(world, goal, metric, show)
            %
            % Show an animation of the distance transform being formed, with a delay
            % of show seconds between frames.


            if nargin < 3
                show = 0;
            end

            if nargin > 1
                dx.goal = goal;
            end

            if isempty(dx.goal)
                error('No goal specified');
            end

            %dx.occgrid(dx.goal(2), dx.goal(1))
            dx.distance = distancexform(dx.occgrid, dx.goal, dx.metric, show);


        end


        function visualize(dx, varargin)
            %DXform.visualize Visualize navigation environment
            %
            % DX.visualize() displays the occupancy grid and the goal distance
            % in a new figure.  The goal distance is shown by intensity which
            % increases with distance from the goal.  Obstacles are overlaid
            % and shown in red.
            %
            % DX.visualize(P) as above but also overlays the points P in the
            % path points which is an Nx2 matrix.
            %
            % See also Navigation.visualize.

            visualize@Navigation(dx, 'distance', dx.distance, varargin{:});

        end

        function n = next(dx, robot)
            directions = [
                -1 -1
                0 -1
                1 -1
                -1 0
                0 0
                1 0
                -1 1
                0 1
                1 1];

            x = robot(1); y = robot(2);
            region = dx.distance(y-1:y+1,x-1:x+1);

            [mn,k] = min(region(:));

            x = x + directions(k,2);
            y = y + directions(k,1);

            if all([x;y] == dx.goal)
                n = [];     % indicate we are at the goal
            else
                n = [x; y];  % else return the next closest point to the goal
            end
            % beyond the edge of the map there be dragons...
            x = max(2, min(numcols(dx.distance)-1, x));
            y = max(2, min(numrows(dx.distance)-1, y));

        end % next

        function visualize3d(dx, p, varargin)
            surf(dx.distance);
            shading interp
            k = sub2ind(size(dx.distance), p(:,2), p(:,1));
            height = dx.distance(k);
            hold on
            if length(varargin) == 0
                varargin{1} = 'k.';
            end
            plot3(p(:,1), p(:,2), height, varargin{:})             
        end
    end % methods
end % classdef
