%DXform Distance transform navigation class
%
% A concrete subclass of the Navigation class that implements the distance 
% transform navigation algorithm which computes minimum distance paths.
%
% Methods::
%
% plan         Compute the cost map given a goal and map
% path         Compute a path to the goal
% visualize    Display the obstacle map (deprecated)
% plot         Display the distance function and obstacle map
% plot3d       Display the distance function as a surface
% display      Print the parameters in human readable form
% char         Convert to string
%
% Properties::
%
% distancemap   The distance transform of the occupancy grid.
% metric        The distance metric, can be 'euclidean' (default) or 'cityblock'
%
% Example::
%
%        load map1           % load map
%        goal = [50,30];     % goal point
%        start = [20, 10];   % start point
%        dx = DXform(map);   % create navigation object
%        dx.plan(goal)       % create plan for specified goal
%        dx.path(start)      % animate path from this start location
%
% Notes::
% - Obstacles are represented by NaN in the distancemap.
% - The value of each element in the distancemap is the shortest distance from the 
%   corresponding point in the map to the current goal.
%
% References::
% -  Robotics, Vision & Control, Sec 5.2.1,
%    Peter Corke, Springer, 2011.
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
        distancemap;   % distance transform results
    end

    methods

        function dx = DXform(world, varargin)
            %DXform.DXform Distance transform constructor
            %
            % DX = DXform(MAP, OPTIONS) is a distance transform navigation object,
            % and MAP is an occupancy grid, a representation of a planar
            % world as a matrix whose elements are 0 (free space) or 1
            % (occupied).
            %
            % Options::
            % 'goal',G      Specify the goal point (2x1)
            % 'metric',M    Specify the distance metric as 'euclidean' (default)
            %               or 'cityblock'.
            % 'inflate',K   Inflate all obstacles by K cells.
            %
            % Other options are supported by the Navigation superclass.
            %
            % See also Navigation.Navigation.

            % TODO NEEDS PROPER ARG HANDLER


            % invoke the superclass constructor
            dx = dx@Navigation(world, varargin{:});

            opt.metric = {'euclidean', 'cityblock'};
            [opt,args] = tb_optparse(opt, varargin);
            dx.metric = opt.metric;


        end

        function s = char(dx)
            %DXform.char Convert to string
            %
            % DX.char() is a string representing the state of the object in 
            % human-readable form.
            %
            % See also DXform.display, Navigation.char
 
            % most of the work is done by the superclass
            s = char@Navigation(dx);

            % dxform specific stuff
            s = char(s, sprintf('  distance metric: %s', dx.metric));
            if ~isempty(dx.distancemap)
                s = char(s, sprintf('  distancemap: computed:'));
            else
                s = char(s, sprintf('  distancemap: empty:'));
            end
        end

        % invoked by superclass on a change of goal, mark the distancemap
        % as invalid
        function goal_change(dx, goal)

            dx.distancemap = [];
            if dx.verbose
                disp('Goal changed -> distancemap cleared');
            end
        end

            
        function plan(dx, goal, show)
            %DXform.plan Plan path to goal
            %
            % DX.plan() updates the internal distancemap where the value of each element is 
            % the minimum distance from the corresponding point to the goal.  The goal is
            % as specified to the constructor.
            %
            % DX.plan(GOAL) as above but uses the specified goal.
            %
            % DX.plan(GOAL, S) as above but displays the evolution of the
            % distancemap, with one iteration displayed every S seconds.
            %
            % Notes::
            % - This may take many seconds.

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
            dx.distancemap = distancexform(dx.occgrid, dx.goal, dx.metric, show);

        end

        function plot(dx, varargin)
            %DXform.plot Visualize navigation environment
            %
            % DX.plot() displays the occupancy grid and the goal distance
            % in a new figure.  The goal distance is shown by intensity which
            % increases with distance from the goal.  Obstacles are overlaid
            % and shown in red.
            %
            % DX.plot(P) as above but also overlays a path given by the set
            % of points P (Mx2).
            %
            % See also Navigation.plot.

            plot@Navigation(dx, 'distance', dx.distancemap, varargin{:});

        end

        function n = next(dx, robot)
            if isempty(dx.distancemap)
                error('No distancemap computed, you need to plan');
            end
            
            % list of all possible directions to move from current cell
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
            
            % find the neighbouring cell that has the smallest distance
            mindist = Inf;
            mindir = [];
            for d=directions'
                % use exceptions to catch attempt to move outside the map
                try
                    if dx.distancemap(y+d(1), x+d(2)) < mindist
                        mindir = d;
                        mindist = dx.distancemap(y+d(1), x+d(2));
                    end
                catch
                end
            end

            x = x + mindir(2);
            y = y + mindir(1);

            if all([x;y] == dx.goal)
                n = [];     % indicate we are at the goal
            else
                n = [x; y];  % else return the next closest point to the goal
            end
        end % next

        function plot3d(dx, p, varargin)
            %DXform.plot3d 3D costmap view
            %
            % DX.plot3d() displays the distance function as a 3D surface with
            % distance from goal as the vertical axis.  Obstacles are "cut out"
            % from the surface.
            %
            % DX.plot3d(P) as above but also overlays a path given by the set
            % of points P (Mx2).
            %
            % DX.plot3d(P, LS) as above but plot the line with the linestyle LS.
            %
            % See also Navigation.plot.
            surf(dx.distancemap);
            shading interp

            if nargin > 1
                % plot path if provided
                k = sub2ind(size(dx.distancemap), p(:,2), p(:,1));
                height = dx.distancemap(k);
                hold on
                if isempty(varargin)
                    varargin{1} = 'k.';
                end
                plot3(p(:,1), p(:,2), height, varargin{:})             
                hold off
            end
        end
    end % methods
end % classdef
