%Bicycle Car-like vehicle class
%
% This concrete class models the kinematics of a car-like vehicle (bicycle
% or Ackerman model) on a plane.  For given steering and velocity inputs it
% updates the true vehicle state and returns noise-corrupted odometry
% readings.
%
% Methods::
%   Bicycle      constructor
%   add_driver   attach a driver object to this vehicle
%   control      generate the control inputs for the vehicle
%   deriv        derivative of state given inputs
%   init         initialize vehicle state
%   f            predict next state based on odometry
%   Fx           Jacobian of f wrt x
%   Fv           Jacobian of f wrt odometry noise
%   update       update the vehicle state
%   run          run for multiple time steps
%   step         move one time step and return noisy odometry
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
%   maxspeed        maximum vehicle speed
%   T               sample interval
%   verbose         verbosity
%   x_hist          history of true vehicle state (Nx3)
%   driver          reference to the driver object
%   x0              initial state, restored on init()
%
% Examples::
%
% Odometry covariance (per timstep) is
%       V = diag([0.02, 0.5*pi/180].^2);
% Create a vehicle with this noisy odometry
%       v = Bicycle( 'covar', diag([0.1 0.01].^2 );
% and display its initial state
%       v 
% now apply a speed (0.2m/s) and steer angle (0.1rad) for 1 time step
%       odo = v.step(0.2, 0.1)
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

classdef Bicycle < Vehicle

    properties
        % state
        L           % length of vehicle
        
        steermax
        accelmax
        vprev
        steerprev
    end

    methods

        function veh = Bicycle(varargin)
        %Bicycle.Bicycle Vehicle object constructor
        %
        % V = Bicycle(OPTIONS)  creates a Bicycle object with the kinematics of a
        % bicycle (or Ackerman) vehicle.
        %
        % Options::
        % 'steermax',M    Maximu steer angle [rad] (default 0.5)
        % 'accelmax',M    Maximum acceleration [m/s2] (default Inf)
        %--
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
        %
        % Notes::
        % - Subclasses the MATLAB handle class which means that pass by reference semantics
        %   apply.
            
            veh = veh@Vehicle(varargin{:});
            
            veh.x = zeros(3,1);

            opt.L = 1;
            opt.steermax = 0.5;
            opt.accelmax = Inf;

            veh = tb_optparse(opt, veh.options, veh);
            veh.vprev = 0;
            veh.x = veh.x0;
        end

        function xnext = f(veh, x, odo, w)
            %Bicycle.f Predict next state based on odometry
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

            dd = odo(1) + w(1); dth = odo(2) + w(2);

            % straightforward code:
            % thp = x(3) + dth;
            % xnext = zeros(1,3);
            % xnext(1) = x(1) + (dd + w(1))*cos(thp);
            % xnext(2) = x(2) + (dd + w(1))*sin(thp);
            % xnext(3) = x(3) + dth + w(2);
            %
            % vectorized code:

            thp = x(:,3) + dth;
            %xnext = x + [(dd+w(1))*cos(thp)  (dd+w(1))*sin(thp) ones(size(x,1),1)*dth+w(2)];
            xnext = x + [dd*cos(thp)  dd*sin(thp) ones(size(x,1),1)*dth];
        end

        function [dx,u] = deriv(veh, t, x, u)
            %Bicycle.deriv  Time derivative of state
            %
            % DX = V.deriv(T, X, U) is the time derivative of state (3x1) at the state
            % X (3x1) with input U (2x1).
            %
            % Notes::
            % - The parameter T is ignored but  called from a continuous time integrator such as ode45 or
            %   Simulink.
            
            % implement acceleration limit if required
            if ~isinf(veh.accelmax)
                if (u(1) - veh.vprev)/veh.dt > veh.accelmax
                    u(1) = veh.vprev + veh.accelmax * veh.dt;
                elseif (u(1) - veh.vprev)/veh.dt < -veh.accelmax
                    u(1) = veh.vprev - veh.accelmax * veh.dt;
                end
                veh.vprev = u(1);
            end
            
            % implement speed and steer angle limits
            u(1) = min(veh.speedmax, max(u(1), -veh.speedmax));
            u(2) = min(veh.steermax, max(u(2), -veh.steermax));

            % compute the derivative
            dx = zeros(3,1);
            dx(1) = u(1)*cos(x(3));
            dx(2) = u(1)*sin(x(3));
            dx(3) = u(1)/veh.L * tan(u(2));
        end
        
        function odo = update(veh, u)
            %Bicycle.update Update the vehicle state
            %
            % ODO = V.update(U) is the true odometry value for
            % motion with U=[speed,steer].
            %
            % Notes::
            % - Appends new state to state history property x_hist.
            % - Odometry is also saved as property odometry.

            % update the state
            dx = veh.dt * veh.deriv([], veh.x, u);
            veh.x = veh.x + dx;

            % compute and save the odometry
            odo = [ norm(dx(1:2)) dx(3) ];
            veh.odometry = odo;

            veh.x_hist = [veh.x_hist; veh.x'];   % maintain history
        end


        function J = Fx(veh, x, odo)
        %Bicycle.Fx  Jacobian df/dx
        %
        % J = V.Fx(X, ODO) is the Jacobian df/dx (3x3) at the state X, for
        % odometry input ODO (1x2) = [distance, heading_change].
        %
        % See also Bicycle.f, Vehicle.Fv.
            dd = odo(1); dth = odo(2);
            thp = x(3) + dth;

            J = [
                1   0   -dd*sin(thp)
                0   1   dd*cos(thp)
                0   0   1
                ];
        end

        function J = Fv(veh, x, odo)
            %Bicycle.Fv  Jacobian df/dv
            %
            % J = V.Fv(X, ODO) is the Jacobian df/dv (3x2) at the state X, for
            % odometry input ODO (1x2) = [distance, heading_change].
            %
            % See also Bicycle.F, Vehicle.Fx.
            dd = odo(1); dth = odo(2);
            thp = x(3);

            J = [
                cos(thp)    0 
                sin(thp)    0 
                0           1
                ];
        end

        function s = char(veh)
        %Bicycle.char Convert to a string
        %
        % s = V.char() is a string showing vehicle parameters and state in 
        % a compact human readable format. 
        %
        % See also Bicycle.display.

            ss = char@Vehicle(veh); 

            s = 'Bicycle object';
            s = char(s, sprintf('  L=%g, steer.max=%g, accel.max=%g', veh.L, veh.steermax, veh.accelmax));
            s = char(s, ss);
        end
    end % method

end % classdef
