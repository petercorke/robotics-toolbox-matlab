%BUG2 Bug navigation class
%
% A concrete subclass of the Navigation class that implements the bug2 
% navigation algorithm.  This is a simple automaton that performs local 
% planning, that is, it can only sense the immediate presence of an obstacle.
%
% Methods::
%   path        Compute a path from start to goal
%   visualize    Display the obstacle map (deprecated)
%   plot         Display the obstacle map
%   display     Display state/parameters in human readable form
%   char        Convert to string
%
% Example::
%    load map1             % load the map
%    bug = Bug2(map);      % create navigation object
%    bug.goal = [50, 35];  % set the goal
%    bug.path([20, 10]);   % animate path to (20,10)
%
% Reference::
% -  Dynamic path planning for a mobile automaton with limited information on the environment,,
%    V. Lumelsky and A. Stepanov, 
%    IEEE Transactions on Automatic Control, vol. 31, pp. 1058-1063, Nov. 1986.
% -  Robotics, Vision & Control, Sec 5.1.2,
%    Peter Corke, Springer, 2011.
%  
% See also Navigation, DXform, Dstar, PRM.

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

classdef Bug2 < Navigation

    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
    end

    methods

        function bug = Bug2(world, goal)
            %Bug2.Bug2 bug2 navigation object constructor
            %
            % B = Bug2(MAP) is a bug2 navigation
            % object, and MAP is an occupancy grid, a representation of a
            % planar world as a matrix whose elements are 0 (free space) or 1
            % (occupied).
            %
            % Options::
            % 'goal',G      Specify the goal point (1x2)
            % 'inflate',K   Inflate all obstacles by K cells.
            %
            % See also Navigation.Navigation.

            % invoke the superclass constructor
            bug = bug@Navigation(world);

            bug.H = [];
            bug.j = 1;
            bug.step = 1;
        end

        % null planning for the bug!
        function plan(bug, goal)
            bug.goal = goal;
        end

        function navigate_init(bug, robot)

            if isempty(bug.goal)
                error('RTB:bug2:nogoal', 'no goal set, cant compute path');
            end
            % parameters of the M-line, direct from initial position to goal
            % as a vector mline, such that [robot 1]*mline = 0
            dims = axis;
            xmin = dims(1); xmax = dims(2);
            ymin = dims(3); ymax = dims(4);

            % create homogeneous representation of the line
            %  line*[x y 1]' = 0
            bug.mline = homline(robot(1), robot(2), ...
                bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            if bug.mline(2) == 0
                % handle the case that the line is vertical
                plot([robot(1) robot(1)], [ymin ymax], 'k--');
            else
                x = [xmin xmax]';
                y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                plot(x, y, 'k--');
            end
        end
        
        % this should be a protected function, but can't make this callable
        % from superclass when it's instantiation of an abstract method
        % (R2010a).
        
        function n = next(bug, robot)
            
            n = [];
            % these are coordinates (x,y)
          
            if bug.step == 1
                % Step 1.  Move along the M-line toward the goal

                if colnorm(bug.goal - robot) == 0 % are we there yet?
                    return
                end

                % motion on line toward goal
                d = bug.goal-robot;
                dx = sign(d(1));
                dy = sign(d(2));

                % detect if next step is an obstacle
                if bug.occgrid(robot(2)+dy, robot(1)+dx)
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j,:) = robot; % define hit point
                    bug.step = 2;
                    % get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgrid==0, robot);
                    bug.k = 2;  % skip the first edge point, we are already there
                else
                    n = robot + [dx; dy];
                end
            end % step 1

            if bug.step == 2
                % Step 2.  Move around the obstacle until we reach a point
                % on the M-line closer than when we started.
                if colnorm(bug.goal-robot) == 0 % are we there yet?
                    return
                end

                if bug.k <= numrows(bug.edge)
                    n = bug.edge(bug.k,:)';  % next edge point
                else
                    % we are at the end of the list of edge points, we
                    % are back where we started.  Step 2.c test.
                    error('robot is trapped')
                    return;
                end

                % are we on the M-line now ?
                if abs( [robot' 1]*bug.mline') <= 0.5
                    bug.message('(%d,%d) moving along the M-line', n);
                    % are closer than when we encountered the obstacle?
                    if colnorm(robot-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
                        % back to moving along the M-line
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        return;
                    end
                end
                % no, keep going around
                bug.message('(%d,%d) keep moving around obstacle', n)
                bug.k = bug.k+1;
            end % step 2
        end % next

    end % methods
end % classdef
