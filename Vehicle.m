%Vehicle Abstract vehicle class
%
% This abstract class models the kinematics of a mobile robot moving on
% a plane and with a pose in SE(2).  For given steering and velocity inputs it
% updates the true vehicle state and returns noise-corrupted odometry
% readings.
%
% Methods::
%   Vehicle          constructor
%   add_driver       attach a driver object to this vehicle
%   control          generate the control inputs for the vehicle
%   f                predict next state based on odometry
%   init             initialize vehicle state
%   run              run for multiple time steps
%   run2             run with control inputs
%   step             move one time step and return noisy odometry
%   update           update the vehicle state
%
% Plotting/display methods::
%   char             convert to string
%   display          display state/parameters in human readable form
%   plot             plot/animate vehicle on current figure
%   plot_xy          plot the true path of the vehicle
%   Vehicle.plotv    plot/animate a pose on current figure
%
% Properties (read/write)::
%   x               true vehicle state: x, y, theta (3x1)
%   V               odometry covariance (2x2)
%   odometry        distance moved in the last interval (2x1)
%   rdim             dimension of the robot (for drawing)
%   L               length of the vehicle (wheelbase)
%   alphalim        steering wheel limit
%   speedmax        maximum vehicle speed
%   T               sample interval
%   verbose         verbosity
%   x_hist          history of true vehicle state (Nx3)
%   driver          reference to the driver object
%   x0              initial state, restored on init()
%
% Examples::
%
% If veh is an instance of a Vehicle class then we can add a driver object
%      veh.add_driver( RandomPath(10) )
% which will move the vehicle within the region -10<x<10, -10<y<10 which we
% can see by
%      veh.run(1000)
% which shows an animation of the vehicle moving for 1000 time steps
% between randomly selected wayoints.
%
% Notes::
% - Subclass of the MATLAB handle class which means that pass by reference semantics
%   apply.
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6
%   Peter Corke,
%   Springer 2011
%
% See also Bicycle, Unicycle, RandomPath, EKF.



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

classdef Vehicle < handle

    properties
        % state
        x           % true state (x,y,theta)
        x_hist      % x history

        % parameters

        speedmax    % maximum speed
        dim         % dimension of the world -dim -> +dim in x and y
        rdim    % dimension of the robot
        dt           % sample interval
        V           % odometry covariance
        odometry    % distance moved in last interval
        verbose
        driver      % driver object
        x0          % initial state
        options
        
        vhandle     % handle to vehicle graphics object
        vtrail      % vehicle trail
    end

    methods(Abstract)
        f
    end
    
    methods

        function veh = Vehicle(varargin)
        %Vehicle Vehicle object constructor
        %
        % V = Vehicle(OPTIONS) creates a Vehicle object that implements the
        % kinematic model of a wheeled vehicle. 
        %
        % Options::
        % 'covar',C       specify odometry covariance (2x2) (default 0)
        % 'speedmax',S    Maximum speed (default 1m/s)
        % 'L',L           Wheel base (default 1m)
        % 'x0',x0         Initial state (default (0,0,0) )
        % 'dt',T          Time interval (default 0.1)
        % 'rdim',R        Robot size as fraction of plot window (default 0.2)
        % 'verbose'       Be verbose
        %
        % Notes::
        % - The covariance is used by a "hidden" random number generator within the class. 
        % - Subclasses the MATLAB handle class which means that pass by reference semantics
        %   apply.
            
          
            % vehicle common
            opt.covar = [];
            opt.rdim = 0.2;
            opt.dt = 0.1;
            opt.x0 = zeros(3,1);
            opt.speedmax = 1;
            opt.vhandle = [];
            
            [opt,args] = tb_optparse(opt, varargin);
            
            veh.V = opt.covar;
            veh.rdim = opt.rdim;
            veh.dt = opt.dt;
            veh.x0 = opt.x0(:);
            assert(isvec(veh.x0, 3), 'Initial configuration must be a 3-vector');
            veh.speedmax = opt.speedmax;
            veh.options = args;  % unused options go back to the subclass
            veh.vhandle = opt.vhandle;
            veh.x_hist = [];
        end

        function init(veh, x0)
            %Vehicle.init Reset state
            %
            % V.init() sets the state V.x := V.x0, initializes the driver 
            % object (if attached) and clears the history.
            %
            % V.init(X0) as above but the state is initialized to X0.
            
            % TODO: should this be called from run?
            
            if nargin > 1
                veh.x = x0(:);
            else
                veh.x = veh.x0;
            end
            veh.x_hist = [];
            
            if ~isempty(veh.driver)
                veh.driver.init();
            end
            
            veh.vhandle = [];
        end

        function yy = path(veh, t, u, y0)
            %Vehicle.path Compute path for constant inputs
            %
            % XF = V.path(TF, U) is the final state of the vehicle (3x1) from the initial
            % state (0,0,0) with the control inputs U (vehicle specific).  TF is  a scalar to 
            % specify the total integration time.
            %
            % XP = V.path(TV, U) is the trajectory of the vehicle (Nx3) from the initial
            % state (0,0,0) with the control inputs U (vehicle specific).  T is a vector (N) of 
            % times for which elements of the trajectory will be computed.
            %
            % XP = V.path(T, U, X0) as above but specify the initial state.
            %
            % Notes::
            % - Integration is performed using ODE45.
            % - The ODE being integrated is given by the deriv method of the vehicle object.
            %
            % See also ODE45.

                if length(t) == 1
                    tt = [0 t];
                else
                    tt = t;
                end

                if nargin < 4
                    y0 = [0 0 0];
                end
                out = ode45( @(t,y) veh.deriv(t, y, u), tt, y0);

                y = out.y';
                if nargout == 0
                    plot(y(:,1), y(:,2));
                    grid on
                    xlabel('X'); ylabel('Y')
                else
                    yy = y;
                    if length(t) == 1
                        % if scalar time given, just return final state
                        yy = yy(end,:);
                    end
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
            if ~isempty(veh.V)
                odo = veh.odometry + randn(1,2)*sqrtm(veh.V);
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
            if isempty(veh.speedmax)
                u(1) = speed;
            else
                u(1) = min(veh.speedmax, max(-veh.speedmax, speed));
            end
            
            % clip the steering angle
            if isempty(veh.steermax)
                u(2) = steer;
            else
                u(2) = max(-veh.steermax, min(veh.steermax, steer));
            end
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
            % See also Vehicle.step, Vehicle.run2.

            if nargin < 2
                nsteps = 1000;
            end
            if ~isempty(veh.driver)
                veh.driver.init()
            end
            %veh.clear();
            if ~isempty(veh.driver)
                veh.driver.plot();
            end

            veh.plot();
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
        % The vehicle is depicted graphically as a narrow triangle that travels
        % "point first" and has a length V.rdim.
        %
        % V.plot(OPTIONS) plots the vehicle on the current axes at a pose given by
        % the current robot state.  If the vehicle has been previously plotted its
        % pose is updated.  
        %
        % V.plot(X, OPTIONS) as above but the robot pose is given by X (1x3).
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
        % 'trail',S    Trail with line style S, use line() name-value pairs
        %
        % Example::
        %          veh.plot('trail', {'Color', 'r', 'Marker', 'o', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 3})

        % Notes::
        % - The last two calls are useful if animating multiple robots in the same
        %   figure.
        %
        % See also Vehicle.plotv, plot_vehicle.


            if isempty(veh.vhandle)
                veh.vhandle = Vehicle.plotv(veh.x, varargin{:});
            end
            
            if ~isempty(varargin) && isnumeric(varargin{1})
                % V.plot(X)
                pos = varargin{1}; % use passed value
            else
                % V.plot()
                pos = veh.x;    % use current state
            end
            
            % animate it
            Vehicle.plotv(veh.vhandle, pos);
            
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
        %Vehicle.char Convert to string
        %
        % s = V.char() is a string showing vehicle parameters and state in 
        % a compact human readable format. 
        %
        % See also Vehicle.display.

            s = '  Superclass: Vehicle';
            s = char(s, sprintf(...
            '    max speed=%g, dT=%g, nhist=%d', ...
                veh.speedmax, veh.dt, ...
                numrows(veh.x_hist)));
            if ~isempty(veh.V)
                s = char(s, sprintf(...
                '    V=(%g, %g)', ...
                    veh.V(1,1), veh.V(2,2)));
            end
            s = char(s, sprintf('    configuration: x=%g, y=%g, theta=%g', veh.x)); 
            if ~isempty(veh.driver)
                s = char(s, '    driven by::');
                s = char(s, [['      '; '      '] char(veh.driver)]);
            end
        end

    end % method

    methods(Static)

        function h = plotv(varargin)
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
        % 'scale',S       Draw vehicle with length S x maximum axis dimension
        % 'size',S        Draw vehicle with length S
        % 'fillcolor',C   Color of vehicle.
        % 'fps',F         Frames per second in animation mode (default 10)
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
        
        if isstruct(varargin{1})
            plot_vehicle(varargin{2}, 'handle', varargin{1});
        else
            h = plot_vehicle(varargin{1}, 'fillcolor', 'b', 'alpha', 0.5);
        end

        end
    end % static methods

end % classdef
