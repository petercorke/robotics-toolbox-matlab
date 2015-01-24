%Vehicle Car-like vehicle class
%
% This class models the kinematics of a car-like vehicle (bicycle model) on
% a plane that moves in SE(2).  For given steering and velocity inputs it
% updates the true vehicle state and returns noise-corrupted odometry
% readings.
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
%   char         convert to string
%
% Class methods::
%   plotv        plot/animate a pose on current figure
%
% Properties (read/write)::
%   x               true vehicle state: x, y, theta (3x1)
%   V               odometry covariance (2x2)
%   odometry        distance moved in the last interval (2x1)
%   rdim             dimension of the robot (for drawing)
%   L               length of the vehicle (wheelbase)
%   alphalim        steering wheel limit
%   maxspeed        maximum vehicle speed
%   T               sample interval
%   verbose         verbosity
%   x_hist          history of true vehicle state (Nx3)
%   driver          reference to the driver object
%   x0              initial state, restored on init()
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
% which shows an animation of the vehicle moving for 1000 time steps
% between randomly selected wayoints.
%
% Notes::
% - Subclasses the MATLAB handle class which means that pass by reference semantics
%   apply.
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6
%   Peter Corke,
%   Springer 2011
%
% See also RandomPath, EKF.


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

classdef Vehicle < handle

    properties
        % state
        x           % true state (x,y,theta)
        x_hist      % x history

        % parameters
        L           % length of vehicle
        alphalim    % steering wheel limit
        maxspeed    % maximum speed
        dim         % dimension of the world -dim -> +dim in x and y
        rdim    % dimension of the robot
        dt           % sample interval
        V           % odometry covariance

        odometry    % distance moved in last interval

        verbose

        driver      % driver object
        x0          % initial state
    end

    methods

        function veh = Vehicle(V, varargin)
        %Vehicle Vehicle object constructor
        %
        % V = Vehicle(V_ACT, OPTIONS)  creates a Vehicle object with actual odometry 
        % covariance V_ACT (2x2) matrix corresponding to the odometry vector [dx dtheta].
        %
        % Options::
        % 'stlim',A       Steering angle limited to -A to +A (default 0.5 rad)
        % 'vmax',S        Maximum speed (default 5m/s)
        % 'L',L           Wheel base (default 1m)
        % 'x0',x0         Initial state (default (0,0,0) )
        % 'dt',T          Time interval
        % 'rdim',R        Robot size as fraction of plot window (default 0.2)
        % 'verbose'       Be verbose
        %
        % Notes::
        % - Subclasses the MATLAB handle class which means that pass by reference semantics
        %   apply.
            
            if ~isnumeric(V)
                error('first arg is V');
            end
            veh.x = zeros(3,1);
            if nargin < 1
                V = zeros(2,2);
            end

            opt.stlim = 0.5;
            opt.vmax = 5;
            opt.L = 1;
            opt.rdim = 0.2;
            opt.dt = 0.1;
            opt.x0 = zeros(3,1);
            opt = tb_optparse(opt, varargin);

            veh.V = V;

            veh.dt = opt.dt;
            veh.alphalim = opt.stlim;
            veh.maxspeed = opt.vmax;
            veh.L = opt.L;
            veh.x0 = opt.x0(:);
            veh.rdim = opt.rdim;
            veh.verbose = opt.verbose;

            veh.x_hist = [];
        end

        function init(veh, x0)
            %Vehicle.init Reset state of vehicle object
            %
            % V.init() sets the state V.x := V.x0, initializes the driver 
            % object (if attached) and clears the history.
            %
            % V.init(X0) as above but the state is initialized to X0.
            if nargin > 1
                veh.x = x0(:);
            else
                veh.x = veh.x0;
            end
            veh.x_hist = [];
            if ~isempty(veh.driver)
                veh.driver.init()
            end
        end

        function add_driver(veh, driver)
            %Vehicle.add_driver Add a driver for the vehicle
            %
            % V.add_driver(D) connects a driver object D to the vehicle.  The driver
            % object has one public method:
            %        [speed, steer] = D.demand();
            % that returns a speed and steer angle.
            %
            % Notes::
            % - The Vehicle.step() method invokes the driver if one is attached.
            %
            % See also Vehicle.step, RandomPath.
            veh.driver = driver;
            driver.veh = veh;
        end

        function xnext = f(veh, x, odo, w)
            %Vehicle.f Predict next state based on odometry
            %
            % XN = V.f(X, ODO) is the predicted next state XN (1x3) based on current
            % state X (1x3) and odometry ODO (1x2) = [distance, heading_change].
            %
            % XN = V.f(X, ODO, W) as above but with odometry noise W.
            %
            % Notes::
            % - Supports vectorized operation where X and XN (Nx3).
            if nargin < 4
                w = [0 0];
            end

            dd = odo(1) + w(1); dth = odo(2);

            % straightforward code:
            % thp = x(3) + dth;
            % xnext = zeros(1,3);
            % xnext(1) = x(1) + (dd + w(1))*cos(thp);
            % xnext(2) = x(2) + (dd + w(1))*sin(thp);
            % xnext(3) = x(3) + dth + w(2);
            %
            % vectorized code:

            thp = x(:,3) + dth;
            xnext = x + [(dd+w(1))*cos(thp)  (dd+w(1))*sin(thp) ones(size(x,1),1)*dth+w(2)];
        end

        function odo = update(veh, u)
            %Vehicle.update Update the vehicle state
            %
            % ODO = V.update(U) is the true odometry value for
            % motion with U=[speed,steer].
            %
            % Notes::
            % - Appends new state to state history property x_hist.
            % - Odometry is also saved as property odometry.

            xp = veh.x; % previous state
            veh.x(1) = veh.x(1) + u(1)*veh.dt*cos(veh.x(3));
            veh.x(2) = veh.x(2) + u(1)*veh.dt*sin(veh.x(3));
            veh.x(3) = veh.x(3) + u(1)*veh.dt/veh.L * u(2);
            odo = [colnorm(veh.x(1:2)-xp(1:2)) veh.x(3)-xp(3)];
            veh.odometry = odo;

            veh.x_hist = [veh.x_hist; veh.x'];   % maintain history
        end


        function J = Fx(veh, x, odo)
        %Vehicle.Fx  Jacobian df/dx
        %
        % J = V.Fx(X, ODO) is the Jacobian df/dx (3x3) at the state X, for
        % odometry input ODO (1x2) = [distance, heading_change].
        %
        % See also Vehicle.f, Vehicle.Fv.
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
            % J = V.Fv(X, ODO) is the Jacobian df/dv (3x2) at the state X, for
            % odometry input ODO (1x2) = [distance, heading_change].
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
            %Vehicle.step Advance one timestep
            %
            % ODO = V.step(SPEED, STEER) updates the vehicle state for one timestep
            % of motion at specified SPEED and STEER angle, and returns noisy odometry.
            %
            % ODO = V.step() updates the vehicle state for one timestep of motion and
            % returns noisy odometry.  If a "driver" is attached then its DEMAND() method
            % is invoked to compute speed and steer angle.  If no driver is attached
            % then speed and steer angle are assumed to be zero.
            %
            % Notes::
            % - Noise covariance is the property V.
            %
            % See also Vehicle.control, Vehicle.update, Vehicle.add_driver.

            % get the control input to the vehicle from either passed demand or driver
            u = veh.control(varargin{:});

            % compute the true odometry and update the state
            odo = veh.update(u);

            % add noise to the odometry
            if veh.V
                odo = veh.odometry + randn(1,2)*veh.V;
            end
        end


        function u = control(veh, speed, steer)
            %Vehicle.control Compute the control input to vehicle
            %
            % U = V.control(SPEED, STEER) is a control input (1x2) = [speed,steer]
            % based on provided controls SPEED,STEER to which speed and steering angle
            % limits have been applied.
            %
            % U = V.control() as above but demand originates with a "driver" object if
            % one is attached, the driver's DEMAND() method is invoked. If no driver is
            % attached then speed and steer angle are assumed to be zero.
            %
            % See also Vehicle.step, RandomPath.
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
            % V.run(N) runs the vehicle model for N timesteps and plots
            % the vehicle pose at each step.
            %
            % P = V.run(N) runs the vehicle simulation for N timesteps and
            % return the state history (Nx3) without plotting.  Each row
            % is (x,y,theta).
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
                if nargout == 0
                    % if no output arguments then plot each step
                    veh.plot();
                    drawnow
                end
            end
            p = veh.x_hist;
        end

        % TODO run and run2 should become superclass methods...

        function p = run2(veh, T, x0, speed, steer)
            %Vehicle.run2 Run the vehicle simulation with control inputs
            %
            % P = V.run2(T, X0, SPEED, STEER) runs the vehicle model for a time T with
            % speed SPEED and steering angle STEER.  P (Nx3) is the path followed and
            % each row is (x,y,theta).
            %
            % Notes::
            % - Faster and more specific version of run() method.
            % - Used by the RRT planner.
            %
            % See also Vehicle.run, Vehicle.step, RRT.
            veh.init(x0);

            for i=1:(T/veh.dt)
                veh.update([speed steer]);
            end
            p = veh.x_hist;
        end

        function h = plot(veh, varargin)
        %Vehicle.plot Plot vehicle
        %
        % V.plot(OPTIONS) plots the vehicle on the current axes at a pose given by
        % the current state.  If the vehicle has been previously plotted its
        % pose is updated.  The vehicle is depicted as a narrow triangle that
        % travels "point first" and has a length V.rdim.
        %
        % V.plot(X, OPTIONS) plots the vehicle on the current axes at the pose X.
        %
        % H = V.plotv(X, OPTIONS) draws a representation of a ground robot as an 
        % oriented triangle with pose X (1x3) [x,y,theta].  H is a graphics handle.
        %
        % V.plotv(H, X) as above but updates the pose of the graphic represented
        % by the handle H to pose X.
        %
        % Options::
        % 'scale',S    Draw vehicle with length S x maximum axis dimension
        % 'size',S     Draw vehicle with length S
        % 'color',C    Color of vehicle.
        % 'fill'       Filled
        %
        % See also Vehicle.plotv.

            h = findobj(gcf, 'Tag', 'Vehicle.plot');
            if isempty(h)
                % no instance of vehicle graphical object found
                h = Vehicle.plotv(veh.x, varargin{:});
                set(h, 'Tag', 'Vehicle.plot');  % tag it
            end
            
            if ~isempty(varargin) && isnumeric(varargin{1})
                % V.plot(X)
                Vehicle.plotv(h, varargin{1}); % use passed value
            else
                % V.plot()
                Vehicle.plotv(h, veh.x);    % use current state
            end
        end

        function out = plot_xy(veh, varargin)
            %Vehicle.plot_xy Plots true path followed by vehicle
            %
            % V.plot_xy() plots the true xy-plane path followed by the vehicle.
            %
            % V.plot_xy(LS) as above but the line style arguments LS are passed
            % to plot.
            %
            % Notes::
            % - The path is extracted from the x_hist property.
            
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
        %Vehicle.verbosity Set verbosity
        %
        % V.verbosity(A) set verbosity to A.  A=0 means silent.
            veh.verbose = v;
        end
            
        function display(nav)
        %Vehicle.display Display vehicle parameters and state
        %
        % V.display() displays vehicle parameters and state in compact 
        % human readable form.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a Vehicle object and the command has no trailing
        %   semicolon.
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
        %Vehicle.char Convert to a string
        %
        % s = V.char() is a string showing vehicle parameters and state in 
        % a compact human readable format. 
        %
        % See also Vehicle.display.

            s = 'Vehicle object';
            s = char(s, sprintf(...
            '  L=%g, maxspeed=%g, alphalim=%g, T=%f, nhist=%d', ...
                veh.L, veh.maxspeed, veh.alphalim, veh.dt, ...
                numrows(veh.x_hist)));
            if ~isempty(veh.V)
                s = char(s, sprintf(...
                '  V=(%g,%g)', ...
                    veh.V(1,1), veh.V(2,2)));
            end
            s = char(s, sprintf('  x=%g, y=%g, theta=%g', veh.x)); 
            if ~isempty(veh.driver)
                s = char(s, '  driven by::');
                s = char(s, [['    '; '    '] char(veh.driver)]);
            end
        end

    end % method

    methods(Static)

        function h_ = plotv(x, varargin)
        %Vehicle.plotv Plot ground vehicle pose
        %
        % H = Vehicle.plotv(X, OPTIONS) draws a representation of a ground robot as an 
        % oriented triangle with pose X (1x3) [x,y,theta].  H is a graphics handle.
        % If X (Nx3) is a matrix it is considered to represent a trajectory in which case
        % the vehicle graphic is animated.
        %
        % Vehicle.plotv(H, X) as above but updates the pose of the graphic represented
        % by the handle H to pose X.
        %
        % Options::
        % 'scale',S    Draw vehicle with length S x maximum axis dimension
        % 'size',S     Draw vehicle with length S
        % 'color',C    Color of vehicle.
        % 'fill'       Filled with solid color as per 'color' option
        % 'fps',F      Frames per second in animation mode (default 10)
        %
        % Example::
        %
        % Generate some path 3xN
        %         p = PRM.plan(start, goal);
        % Set the axis dimensions to stop them rescaling for every point on the path
        %         axis([-5 5 -5 5]);
        %
        % Now invoke the static method
        %         Vehicle.plotv(p);
        %
        % Notes::
        % - This is a class method.
        %
        % See also Vehicle.plot.

            if isscalar(x) && ishandle(x)
                % plotv(h, x)
                h = x;
                x = varargin{1};
                x = x(:)';
                T = transl([x(1:2) 0]) * trotz( x(3) );
                set(h, 'Matrix', T);
                return
            end

            opt.scale = 1/60;
            opt.size = [];
            opt.fill = false;
            opt.color = 'r';
            opt.fps = 10;
            
            [opt,args] = tb_optparse(opt, varargin);

            lineprops = { 'Color', opt.color' };
            if opt.fill
                lineprops = [lineprops 'fill' opt.color ];
            end
            
            
            % compute the dimensions of the robot
            if ~isempty(opt.size)
                d = opt.size;
            else
                % get the current axes dimensions
                a = axis;
                d = (a(2)+a(4) - a(1)-a(3)) * opt.scale;
            end
            
            % draw it
            points = [
                d 0
                -d -0.6*d
                -d 0.6*d
            ]';
            
            h = hgtransform();
            hp = plot_poly(points, lineprops{:});
            for hh=hp
                set(hh, 'Parent', h);
            end

            if (numel(x) > 3) && (numcols(x) == 3)
                % animation mode
                for i=1:numrows(x)
                    T = transl([x(i,1:2) 0]) * trotz( x(i,3) );
                    set(h, 'Matrix', T);
                    pause(1/opt.fps);
                end
            elseif (numel(x) == 3)
                % compute the pose
                % convert vector form of pose to SE(3)
            
                x = x(:)';
                T = transl([x(1:2) 0]) * trotz( x(3) );
                set(h, 'Matrix', T);
            else
                error('bad pose');
            end

            if nargout > 0
                h_ = h;
            end
        end

    end % static methods

end % classdef
