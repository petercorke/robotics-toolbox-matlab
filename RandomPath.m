%RandomPath Vehicle driver class
%
% Create a "driver" object capable of steering a Vehicle object through random 
% waypoints within a rectangular region and at constant speed.
%
% The driver object is connected to a Vehicle object by the latter's
% add_driver() method.  The driver's demand() method is invoked on every
% call to the Vehicle's step() method.
%
% Methods::
%  init       reset the random number generator
%  demand     return speed and steer angle to next waypoint
%  display    display the state and parameters in human readable form
%  char       convert to string
%      
% Properties::
%  goal          current goal/waypoint coordinate
%  veh           the Vehicle object being controlled
%  dim           dimensions of the work space (2x1) [m]
%  speed         speed of travel [m/s]
%  closeenough   proximity to waypoint at which next is chosen [m]
%
% Example::
%
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%
% Notes::
% - It is possible in some cases for the vehicle to move outside the desired
%   region, for instance if moving to a waypoint near the edge, the limited
%   turning circle may cause the vehicle to temporarily move outside.
% - The vehicle chooses a new waypoint when it is closer than property
%   closeenough to the current waypoint.
% - Uses its own random number stream so as to not influence the performance
%   of other randomized algorithms such as path planning.
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
% See also Vehicle.


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

% TODO
%  should be a subclass of VehicleDriver
%  Vehicle should be an abstract superclass
%  dim should be checked, can be a 4-vector like axis()

classdef RandomPath < handle
    properties
        goal        % current goal
        h_goal      % graphics handle for goal
        veh         % the vehicle we are driving
        dim
        speed       % speed of travel
        dthresh  % proximity to goal before
        d_prev
        randstream  % random stream just for Sensors
        show
        verbose
    end

    methods

        function driver = RandomPath(dim, varargin)
        %RandomPath.RandomPath Create a driver object
        %
        % D = RandomPath(DIM, OPTIONS) returns a "driver" object capable of driving 
        % a Vehicle object through random waypoints.  The waypoints are positioned 
        % inside a rectangular region bounded by +/- DIM in the x- and y-directions.
        %
        % Options::
        % 'speed',S      Speed along path (default 1m/s).
        % 'dthresh',D    Distance from goal at which next goal is chosen.
        %
        % See also Vehicle.

        % TODO options to specify region, maybe accept a Map object?
        % dim can be a 4-vector

            driver.dim = dim;

            opt.speed = 1;
            opt.dthresh = 0.05 * dim;
            opt.show = true;
            
            driver = tb_optparse(opt, varargin, driver);

            drive.d_prev = Inf;
            driver.randstream = RandStream.create('mt19937ar');
        end

        function init(driver)
        %RandomPath.init Reset random number generator
        %
        % R.init() resets the random number generator used to create the waypoints.
        % This enables the sequence of random waypoints to be repeated.
        %
        % See also RANDSTREAM.
            driver.goal = [];
            driver.randstream.reset();
            driver.h_goal = [];
        end

                % called by Vehicle superclass
        function visualize(driver)
            clf
            d = driver.dim;
            axis([-d d -d d]);
            hold on
            xlabel('x');
            ylabel('y');
        end

        % private method, invoked from demand() to compute a new waypoint
        function setgoal(driver)
            
           % choose a uniform random goal within inner 80% of driving area
           while true
               r = driver.randstream.rand(2,1);
               driver.goal = 0.8 * driver.dim * (r - 0.5)*2;
               if norm(driver.goal - driver.veh.x(1:2)) > 2*driver.dthresh
                   break;
               end
           end
           
           if driver.verbose
               fprintf('set goal: (%.1f %.1f)\n', driver.goal);
           end
           if driver.show && isempty(driver.h_goal)
               driver.h_goal = plot(driver.goal(1), driver.goal(2), 'rd', 'MarkerSize', 12, 'MarkerFaceColor', 'r')
           else
               set(driver.h_goal, 'Xdata', driver.goal(1), 'Ydata', driver.goal(2))
           end
        end

        function [speed, steer] = demand(driver)
        %RandomPath.demand Compute speed and heading to waypoint
        %
        % [SPEED,STEER] = R.demand() returns the speed and steer angle to
        % drive the vehicle toward the next waypoint.  When the vehicle is
        % within R.closeenough a new waypoint is chosen.
        %
        % See also Vehicle.
            if isempty(driver.goal)
                driver.setgoal()
            end

            speed = driver.speed;

            goal_heading = atan2(driver.goal(2)-driver.veh.x(2), ...
                driver.goal(1)-driver.veh.x(1));
            d_heading = angdiff(goal_heading, driver.veh.x(3));

            steer = d_heading;

            % if nearly at goal point, choose the next one
            d = norm(driver.veh.x(1:2) - driver.goal);
            if d < driver.dthresh
                driver.setgoal();
            elseif d > 2*driver.d_prev
                driver.setgoal();
            end
            driver.d_prev = d;
        end

        function display(driver)
        %RandomPath.display Display driver parameters and state
        %
        % R.display() displays driver parameters and state in compact 
        % human readable form.
        %
        % See also RandomPath.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(driver) );
        end % display()

        function s = char(driver)
        %RandomPath.char Convert to string
        %
        % s = R.char() is a string showing driver parameters and state in in 
        % a compact human readable format. 
            s = 'RandomPath driver object';
            s = char(s, sprintf('  current goal=(%g,%g), dimension %g, dthresh %g', ...
                driver.goal, driver.dim, driver.dthresh));
        end

    end % methods
end % classdef
