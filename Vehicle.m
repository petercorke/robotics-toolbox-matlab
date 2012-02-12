%Vehicle Car-like vehicle class
%
% This class models the kinematics of a car-like vehicle (bicycle model).  For
% given steering and velocity inputs it updates the true vehicle state and returns
% noise-corrupted odometry readings.
%
%  veh = Vehicle(V) creates a Vehicle object with odometry covariance V, where V is
%  a 2x2 matrix corresponding to the odometry vector [dx dtheta].
%
% Methods::
%   init         initialize vehicle state
%   f            predict next state based on odometry
%   step         move one time step and return noisy odometry
%   control      generate the control inputs for the vehicle
%   update       update the vehicle state
%   run          run for multiple time steps
%   Fx           Jacobian of f wrt x
%   Fv           Jacobian of f wrt odometry noise
%   gstep        like step() but displays vehicle
%   plot         plot/animate vehicle on current figure
%   plot_xy      plot the true path of the vehicle
%   add_driver   attach a driver object to this vehicle
%   display      display state/parameters in human readable form
%   char         convert state/parameters to human readable string
%
% Properties (read/write)::
%   x               true vehicle state 3x1
%   V               odometry covariance
%   odometry        distance moved in the last interval
%   dim             dimension of the robot's world
%   robotdim        dimension of the robot (for drawing)
%   L               length of the vehicle (wheelbase)
%   alphalim        steering wheel limit
%   maxspeed        maximum vehicle speed
%   T               sample interval
%   verbose         verbosity
%   x_hist          history of true vehicle state Nx3
%   driver          reference to the driver object
%   x0              initial state, init() sets x := x0
%
% Examples::
%
% Create a vehicle with odometry covariance
%       v = Vehicle( diag([0.1 0.01].^2 );
% and display its initial state
%       v 
% now apply a speed (0.2m/s) and steer angle (0.1rad) for 1 time step
%       odo = v.update([0.2, 0.1])
% where odo is the noisy odometry estimate, and the new true vehicle state
%       v
%
% We can add a driver object
%      v.add_driver( RandomPath(10) )
% which will move the vehicle within the region -10<x<10, -10<y<10 which we
% can see by
%      v.run(1000)
% which will show an animation of the vehicle moving between randomly
% selected wayoints.
%
% Reference::
%
%   Robotics, Vision & Control,
%   Peter Corke,
%   Springer 2011
%
% See also RandomPath, EKF.

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

classdef Vehicle < handle

    properties
        x           % true state (x,y,theta)
        V           % odometry covariance
        odometry    % distance moved in last interval

        dim         % dimension of the world -dim -> +dim in x and y
        robotdim    % dimension of the robot
        L           % length of vehicle

        alphalim    % steering wheel limit
        maxspeed    % maximum speed

        T           % sample interval
        verbose


        x_hist          % x history
        driver      % driver object
        x0
    end

    methods

        function veh = Vehicle(V)
        %Vehicle Vehicle object constructor
        %
        % V = Vehicle(VACT)  creates a Vehicle object with actual odometry 
        % covariance VACT, where VACT is a 2x2 matrix corresponding to the 
        % odometry vector [dx dtheta].
        %
        % Default parameters are:
        %   alphalim   0.5
        %   maxspeed   5
        %   L          1
        %   robotdim   0.2
        %   x0         (0,0,0)
        %
        % and can be overridden by assigning properties after the object has 
        % been created.
            
            veh.x = zeros(3,1);
            veh.T = 0.1;
            if nargin < 1
                V = zeros(2,2);
            end
            veh.V = V;
            veh.alphalim = 0.5;
            veh.maxspeed = 5;
            veh.L = 1;
            veh.verbose = false;
            veh.x_hist = [];
            veh.robotdim = 0.2;
            veh.x0 = zeros(3,1);
        end

        function init(veh)
            %Vehicle.init Reset state of vehicle object
            %
            % V.init() sets the state V.x := V.x0
            veh.x = veh.x0;
            veh.x_hist = [];
            if ~isempty(veh.driver)
                veh.driver.init()
            end
        end

        function add_driver(veh, driver)
            %Vehicle.add_driver Add a driver for the vehicle
            %
            % V.add_driver(D) adds a driver object D for the vehicle.  The driver
            % object has one public method:
            %   [speed, steer] = D.demand();
            % that returns a speed and steer angle.
            %
            % See also RandomPath.
            veh.driver = driver;
            driver.veh = veh;
        end

        function xnext = f(veh, x, odo, w)
            %Vehicle.f Predict next state based on odometry
            %
            % XN = V.f(X, ODO) predict next state XN based on current state X and
            % odometry ODO. X is 3x1, ODO is [distance,change_heading].
            %
            % XN = V.f(X, ODO, W) predict next state XN based on current state X,
            % odometry ODO, and odometry noise W.
            if nargin < 4
                w = [0 0];
            end

            dd = odo(1) + w(1); dth = odo(2);
            thp = x(3) + dth;
            xnext = zeros(3,1);
            xnext(1) = x(1) + (dd + w(1))*cos(thp);
            xnext(2) = x(2) + (dd + w(1))*sin(thp);
            xnext(3) = x(3) + dth + w(2);
        end

        function odo = update(veh, u)
            %Vehicle.update Update the vehicle state
            %
            % ODO = V.update(U) returns noisy odometry readings (covariance V.V) for
            % motion with U=[speed,steer].
            xp = veh.x;
            veh.x(1) = veh.x(1) + u(1)*veh.T*cos(veh.x(3));
            veh.x(2) = veh.x(2) + u(1)*veh.T*sin(veh.x(3));
            veh.x(3) = veh.x(3) + u(1)*veh.T/veh.L * u(2);
            odo = [norm2(veh.x(1:2)-xp(1:2)) veh.x(3)-xp(3)];
            veh.odometry = odo;

            veh.x_hist = [veh.x_hist; veh.x'];   % maintain history
        end


        function J = Fx(veh, x, odo)
        %Vehicle.Fx  Jacobian df/dx
        %
        % J = V.Fx(X, ODO) returns the Jacobian df/dx at the state X, for
        % odometry input ODO.  J is 3x3.
        %
        % See also Vehicle.F, Vehicle.Fv.
            dd = odo(1); dth = odo(2);
            thp = x(3) + dth;

            J = [
                1   0   -dd*sin(thp)
                0   1   dd*cos(thp)
                0   0   1
                ];
        end

        function J = Fv(veh, x, odo)
            %Vehicle.Fv  Jacobian df/dv
            %
            % J = V.Fv(X, ODO) returns the Jacobian df/dv at the state X, for
            % odometry input ODO.  J is 3x2.
            %
            % See also Vehicle.F, Vehicle.Fx.
            dd = odo(1); dth = odo(2);
            thp = x(3) + dth;

            J = [
                cos(thp)    -dd*sin(thp)
                sin(thp)    dd*cos(thp)
                0           1
                ];
        end


        function odo = step(veh, varargin)
            %Vehicle.step Move the vehicle model ahead one time step
            %
            % ODO = V.step(SPEED, STEER) updates the vehicle state for one timestep
            % of motion at specified SPEED and STEER angle, and returns noisy odometry.
            %
            % ODO = V.step() updates the vehicle state for one timestep of motion and
            % returns noisy odometry.  If a "driver" is attached then its DEMAND() method
            % is invoked to compute speed and steer angle.  If no driver is attached
            % then speed and steer angle are assumed to be zero.
            %
            % See also Vehicle.control, Vehicle.update, Vehicle.add_driver.
            u = veh.control(varargin{:});
            odo = veh.update(u);
            %veh.showrobot();
            %drawnow

            if veh.V
                odo = veh.odometry + randn(1,2)*veh.V;
            end
        end

        function odo = gstep(veh, varargin)
            odo = veh.step(varargin{:});
            veh.showrobot();
            drawnow
        end

        function u = control(veh, speed, steer)
            %Vehicle.control Compute the control input to vehicle
            %
            % U = V.control(SPEED, STEER) returns a control input (speed,steer)
            % based on provided controls SPEED,STEER to which speed and steering
            % angle limits have been applied.
            %
            % U = V.control() returns a control input (speed,steer) from a "driver"
            % if one is attached, the driver's DEMAND() method is invoked. If no driver is attached
            % then speed and steer angle are assumed to be zero.
            %
            % See also RandomPath.
            if nargin < 2
                % if no explicit demand, and a driver is attached, use
                % it to provide demand
                if ~isempty(veh.driver)
                    [speed, steer] = veh.driver.demand();
                else
                    % no demand, do something safe
                    speed = 0;
                    steer = 0;
                end
            end

            % clip the speed
            u(1) = min(veh.maxspeed, max(-veh.maxspeed, speed));

            % clip the steering angle
            u(2) = max(-veh.alphalim, min(veh.alphalim, steer));
        end

        function p = run(veh, nsteps)
            %Vehicle.run Run the vehicle simulation
            %
            % V.run(N) run the vehicle simulation for N timesteps.
            %
            % P = V.run(N) run the vehicle simulation for N timesteps and
            % return the state history as an Nx3 matrix.
            %
            % See also Vehicle.step.

            if nargin < 2
                nsteps = 1000;
            end

            %veh.clear();
            if ~isempty(veh.driver)
                veh.driver.visualize();
            end
            veh.visualize();
            for i=1:nsteps
                veh.step();
            end
            p = veh.x_hist;
        end

        function h = plot(veh, x)
            %Vehicle.plot Plot vehicle
            %
            % V.plot() plots the vehicle on the current axes at a pose given by
            % the current state.  If the vehicle has been previously plotted its
            % pose is updated.  The vehicle is depicted as a narrow triangle that
            % travels "point first" and has a length V.robotdim.
            %
            % V.plot(X) plots the vehicle on the current axes at the pose X.
            h = findobj(gcf, 'Tag', 'mobilerobot');
            if isempty(h)
                h = line(0, 0, 'Tag', 'mobilerobot');
            end

            d = veh.robotdim;
            points = [
                d 0 1
                -d -0.6*d 1
                -d 0.6*d 1
                d 0 1];
            if nargin < 2
                x = veh.x;
            end
            T = transl([x(1:2)' 0]) * trotz(x(3));
            T(:,3) = [];
            T(3,:) = [];
            points = T * points';
            set(h, 'Xdata', points(1,:), 'Ydata', points(2,:), ...
                'Zdata', repmat(veh.x(3), 1, 4) );
        end


        function out = plot_xy(veh, varargin)
            %Vehicle.plot_xy Plot true path followed by vehicle
            %
            % V.plot_xy() plots the true xy-plane path followed by the vehicle.
            %
            % V.plot_xy(LS) as above but the line style arguments LS are passed
            % to plot.
            
            xyt = veh.x_hist;
            if nargout == 0
                plot(xyt(:,1), xyt(:,2), varargin{:});
            else
                out = xyt;
            end
        end

        function visualize(veh)
            grid on
        end

        function verbosity(veh, v)
            veh.verbose = v;
        end
            
        function display(nav)
        %Vehicle.display Display vehicle parameters and state
        %
        % V.display() display vehicle parameters and state in compact 
        % human readable form.
        %
        % See also Vehicle.char.

            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(nav) );
        end % display()

        function s = char(veh)
        %Vehicle.char Convert vehicle parameters and state to a string
        %
        % s = V.char() is a string showing vehicle parameters and state in in 
        % a compact human readable format. 

            s = 'Vehicle object';
            s = strvcat(s, sprintf(...
            '  L=%g, maxspeed=%g, alphalim=%g, T=%f, nhist=%d', ...
                veh.L, veh.maxspeed, veh.alphalim, veh.T, ...
                numrows(veh.x_hist)));
            if ~isempty(veh.V)
                s = strvcat(s, sprintf(...
                '  V=(%g,%g)', ...
                    veh.V(1,1), veh.V(2,2)));
            end
            s = strvcat(s, sprintf('  x=%g, y=%g, theta=%g', veh.x)); 
            if ~isempty(veh.driver)
                s = strvcat(s, '  driven by::');
                s = strvcat(s, [['    '; '    '] char(veh.driver)]);
            end
        end

    end % method

end % classdef
