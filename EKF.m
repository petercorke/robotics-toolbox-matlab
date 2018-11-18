%EKF Extended Kalman Filter for navigation
%
% Extended Kalman filter for optimal estimation of state from noisy
% measurments given a non-linear dynamic model.  This class is specific to
% the problem of state estimation for a vehicle moving in SE(2).
%
% This class can be used for:
%   - dead reckoning localization
%   - map-based localization
%   - map making
%   - simultaneous localization and mapping (SLAM)
%
% It is used in conjunction with:
%   - a kinematic vehicle model that provides odometry output, represented 
%     by a Vehicle sbuclass object.  
%   - The vehicle must be driven within the area of the map and this is 
%     achieved by connecting the Vehicle subclass object to a Driver object.  
%   - a map containing the position of a number of landmark points and is
%     represented by a LandmarkMap object.
%   - a sensor that returns measurements about landmarks relative to the 
%     vehicle's pose and is represented by a Sensor object subclass.
%
% The EKF object updates its state at each time step, and invokes the 
% state update methods of the vehicle object.  The complete history of estimated
% state and covariance is stored within the EKF object.
%
% Methods::
%   run            run the filter
%   plot_xy        plot the actual path of the vehicle
%   plot_P         plot the estimated covariance norm along the path
%   plot_map       plot estimated landmark points and confidence limits
%   plot_vehicle   plot estimated vehicle covariance ellipses
%   plot_error     plot estimation error with standard deviation bounds
%   display        print the filter state in human readable form
%   char           convert the filter state to human readable string
%
% Properties::
%  x_est      estimated state
%  P          estimated covariance
%  V_est      estimated odometry covariance
%  W_est      estimated sensor covariance
%  landmarks   maps sensor landmark id to filter state element
%  robot      reference to the Vehicle object
%  sensor     reference to the Sensor subclass object
%  history    vector of structs that hold the detailed filter state from
%             each time step
%  verbose    show lots of detail (default false)
%  joseph     use Joseph form to represent covariance (default true)
%
% Vehicle position estimation (localization)::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a Kalman filter with estimated covariance V_est and initial
% state covariance P0
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    ekf = EKF(veh, V_est, P0);
% We run the simulation for 1000 time steps
%    ekf.run(1000);
% then plot true vehicle path
%    veh.plot_xy('b');
% and overlay the estimated path
%    ekf.plot_xy('r');
% and overlay uncertainty ellipses
%    ekf.plot_ellipse('g');
% We can plot the covariance against time as
%    clf
%    ekf.plot_P();
%
% Map-based vehicle localization::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a map with 20 point landmarks, create a sensor that uses the map 
% and vehicle state to estimate landmark range and bearing with covariance
% W, the Kalman filter with estimated covariances V_est and W_est and initial
% vehicle state covariance P0
%    veh = Bicycle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    map = LandmarkMap(20);
%    sensor = RangeBearingSensor(veh, map, W);
%    ekf = EKF(veh, V_est, P0, sensor, W_est, map);
% We run the simulation for 1000 time steps
%    ekf.run(1000);
% then plot the map and the true vehicle path
%    map.plot();
%    veh.plot_xy('b');
% and overlay the estimatd path
%    ekf.plot_xy('r');
% and overlay uncertainty ellipses
%    ekf.plot_ellipse('g');
% We can plot the covariance against time as
%    clf
%    ekf.plot_P();
%
% Vehicle-based map making::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a sensor that uses the map and vehicle state to estimate landmark range 
% and bearing with covariance W, the Kalman filter with estimated sensor 
% covariance W_est and a "perfect" vehicle (no covariance),
% then run the filter for N time steps.
%
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    map = LandmarkMap(20);
%    sensor = RangeBearingSensor(veh, map, W);
%    ekf = EKF(veh, [], [], sensor, W_est, []);
% We run the simulation for 1000 time steps
%    ekf.run(1000);
% Then plot the true map
%    map.plot();
% and overlay the estimated map with 97% confidence ellipses
%    ekf.plot_map('g', 'confidence', 0.97);
%
% Simultaneous localization and mapping (SLAM)::
%
% Create a vehicle with odometry covariance V, add a driver to it,
% create a map with 20 point landmarks, create a sensor that uses the map 
% and vehicle state to estimate landmark range and bearing with covariance
% W, the Kalman filter with estimated covariances V_est and W_est and initial
% state covariance P0, then run the filter to estimate the vehicle state at 
% each time step and the map.
%
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    map = PointMap(20);
%    sensor = RangeBearingSensor(veh, map, W);
%    ekf = EKF(veh, V_est, P0, sensor, W, []);
% We run the simulation for 1000 time steps
%    ekf.run(1000);
% then plot the map and the true vehicle path
%    map.plot();
%    veh.plot_xy('b');
% and overlay the estimated path
%    ekf.plot_xy('r');
% and overlay uncertainty ellipses
%    ekf.plot_ellipse('g');
% We can plot the covariance against time as
%    clf
%    ekf.plot_P();
% Then plot the true map
%    map.plot();
% and overlay the estimated map with 3 sigma ellipses
%    ekf.plot_map(3, 'g');
%
% References::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
%   Stochastic processes and filtering theory,
%   AH Jazwinski
%   Academic Press 1970
%
% Acknowledgement::
% 
% Inspired by code of Paul Newman, Oxford University, 
% http://www.robots.ox.ac.uk/~pnewman
%
% See also Vehicle, RandomPath, RangeBearingSensor, PointMap, ParticleFilter.



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
classdef EKF < handle

    %TODO
    % add a hook for data association
    % show landmark covar as ellipse or pole
    % show vehicle covar as ellipse
    % show track
    % landmarks property should be an array of structs
    
    properties
        % STATE:
        % the state vector is [x_vehicle x_map] where
        % x_vehicle is 1x3 and
        % x_map is 1x(2N) where N is the number of map landmarks
        x_est           % estimated state
        P_est           % estimated covariance

        % landmarks keeps track of landmarks we've seen before.
        % Each column represents a landmark.  This is a fixed size
        % array, indexed by landmark id.
        % row 1: the start of this landmark's state in the state vector, initially NaN
        % row 2: the number of times we've sighted the landmark
        landmarks           % map state

        V_est           % estimate of covariance V
        W_est           % estimate of covariance W

        robot           % reference to the robot vehicle
        sensor          % reference to the sensor

        % FLAGS:
        %   estVehicle    estMap
        %        0          0     
        %        0          1     make map
        %        1          0     dead reckoning
        %        1          1     SLAM
        estVehicle      % flag: estimating vehicle location
        estMap          % flag: estimating map

        joseph          % flag: use Joseph form to compute p
        verbose
        keepHistory     % keep history
        P0              % passed initial covariance
        map             % passed map

        % HISTORY:
        % vector of structs to hold EKF history
        % .x_est estimated state
        % .odo   vehicle odometry
        % .P     estimated covariance matrix
        % .innov innovation
        % .S     
        % .K     Kalman gain matrix
        history
        dim          % robot workspace dimensions
    end

    methods

        % constructor
        function ekf = EKF(robot, V_est, P0, varargin)
            %EKF.EKF EKF object constructor
            %
            % E = EKF(VEHICLE, V_EST, P0, OPTIONS) is an EKF that estimates the state
            % of the VEHICLE (subclass of Vehicle) with estimated odometry covariance V_EST (2x2) and
            % initial covariance (3x3).
            %
            % E = EKF(VEHICLE, V_EST, P0, SENSOR, W_EST, MAP, OPTIONS) as above but
            % uses information from a VEHICLE mounted sensor, estimated
            % sensor covariance W_EST and a MAP (LandmarkMap class).
            %
            % Options::
            % 'verbose'      Be verbose.
            % 'nohistory'    Don't keep history.
            % 'joseph'       Use Joseph form for covariance
            % 'dim',D        Dimension of the robot's workspace.
            %                - D scalar; X: -D to +D, Y: -D to +D
            %                - D (1x2); X: -D(1) to +D(1), Y: -D(2) to +D(2)
            %                - D (1x4); X: D(1) to D(2), Y: D(3) to D(4)
            %
            % Notes::
            % - If MAP is [] then it will be estimated.
            % - If V_EST and P0 are [] the vehicle is assumed error free and
            %   the filter will only estimate the landmark positions (map).
            % - If V_EST and P0 are finite the filter will estimate the
            %   vehicle pose and the landmark positions (map).
            % - EKF subclasses Handle, so it is a reference object.
            % - Dimensions of workspace are normally taken from the map if given.
            %
            % See also Vehicle, Bicycle, Unicycle, Sensor, RangeBearingSensor, LandmarkMap.

            opt.history = true;
            opt.joseph = true;
            opt.dim = [];
            
            [opt,args] = tb_optparse(opt, varargin);
            
            % copy options to class properties
            ekf.verbose = opt.verbose;
            ekf.keepHistory = opt.history;
            ekf.joseph = opt.joseph;
            ekf.P0 = P0;
            ekf.dim = opt.dim;         
            
            % figure what we need to estimate
            ekf.estVehicle = false;
            ekf.estMap = false;
            switch length(args)
                case 0
                    % Deadreckoning:
                    %    E = EKF(VEHICLE, V_EST, P0, OPTIONS)
                    sensor = []; W_est = []; map = [];
                    ekf.estVehicle = true;
                case 3
                    % Using a map:
                    %    E = EKF(VEHICLE, V_EST, P0, SENSOR, W_EST, MAP, OPTIONS)
                    % Estimating a map:
                    %    E = EKF(VEHICLE,[], [], SENSOR, W_EST, [], OPTIONS)
                    % Full SLAM:
                    %    E = EKF(VEHICLE, V_EST, P0, SENSOR, W_EST, [], OPTIONS)

                    [sensor, W_est, map] = deal(args{:});
                    if isempty(map)
                        ekf.estMap = true;
                    end
                    if ~isempty(V_est)
                        ekf.estVehicle = true;
                    end
                    
                otherwise
                    error('RTB:EKF:badarg', 'incorrect number of non-option arguments');
            end
            
            % check types for passed objects
            if ~isempty(map) && ~isa(map, 'LandmarkMap')
                error('RTB:EKF:badarg', 'expecting LandmarkMap object');
            end
            if ~isempty(sensor) && ~isa(sensor, 'Sensor')
                error('RTB:EKF:badarg', 'expecting Sensor object');
            end
            if ~isa(robot, 'Vehicle')
                error('RTB:EKF:badarg', 'expecting Vehicle object');
            end
            
            % copy arguments to class properties
            ekf.robot = robot;
            ekf.V_est = V_est;
            ekf.sensor = sensor;
            ekf.map = map;
            ekf.W_est = W_est;
            
            ekf.init();
        end

        function init(ekf)
        %EKF.init Reset the filter
        %
        % E.init() resets the filter state and clears landmarks and history.
            ekf.robot.init();

            % clear the history
            ekf.history = [];
            
            if isempty(ekf.V_est)
                % perfect vehicle case
                ekf.estVehicle = false;
                ekf.x_est = [];
                ekf.P_est = [];
            else
                % noisy odometry case
                ekf.x_est = ekf.robot.x(:);   % column vector
                ekf.P_est = ekf.P0;
                ekf.estVehicle = true;
            end
            
            if ~isempty(ekf.sensor)
                ekf.landmarks = NaN*zeros(2, ekf.sensor.map.nlandmarks);
            end
            
        end

        function run(ekf, n, varargin)
        %EKF.run Run the filter
        %
        % E.run(N, OPTIONS) runs the filter for N time steps and shows an animation
        % of the vehicle moving.
        %
        % Options::
        % 'plot'     Plot an animation of the vehicle moving
        %
        % Notes::
        % - All previously estimated states and estimation history are initially
        %   cleared.
        
            opt.plot = true;
            opt.movie = [];
            opt = tb_optparse(opt, varargin);
            
            ekf.init();
            
            if opt.plot
                if ~isempty(ekf.sensor)
                    ekf.sensor.map.plot();
                elseif ~isempty(ekf.dim)
                    switch length(ekf.dim)
                        case 1
                            d = ekf.dim;
                            axis([-d d -d d]);
                        case 2
                            w = ekf.dim(1), h = ekf.dim(2);
                            axis([-w w -h h]);
                        case 4
                            axis(ekf.dim);
                    end
                    set(gca, 'ALimMode', 'manual');
                else
                    opt.plot = false;
                end
                axis manual
                xlabel('X'); ylabel('Y')
            end


            % simulation loop
            anim = Animate(opt.movie);
            for k=1:n
                
                if opt.plot
                    ekf.robot.plot();
                    drawnow
                end
                
                ekf.step(opt);
                anim.add();
            end
            
            anim.close();
        end
        
        function xyt = get_xy(ekf)
        %EKF.plot_xy Get vehicle position
        %
        % P = E.get_xy() is the estimated vehicle pose trajectory
        % as a matrix (Nx3) where each row is x, y, theta.
        %
        % See also EKF.plot_xy, EKF.plot_error, EKF.plot_ellipse, EKF.plot_P.
        
            if ekf.estVehicle
                xyt = zeros(length(ekf.history), 3);
                for i=1:length(ekf.history)
                    h = ekf.history(i);
                    xyt(i,:) = h.x_est(1:3)';
                end
            else
                xyt = [];
            end
        end

        function out = plot_xy(ekf, varargin)
            %EKF.plot_xy Plot vehicle position
            %
            % E.plot_xy() overlay the current plot with the estimated vehicle path in
            % the xy-plane.
            %
            % E.plot_xy(LS) as above but the optional line style arguments
            % LS are passed to plot.
            %
            % See also EKF.get_xy, EKF.plot_error, EKF.plot_ellipse, EKF.plot_P.
            
            xyt=ekf.get_xy();
            plot(xyt(:,1), xyt(:,2), varargin{:});
            plot(xyt(1,1), xyt(1,2), 'ko', 'MarkerSize', 8, 'LineWidth', 2);
        end
        
        function out = plot_error(ekf, varargin)
        %EKF.plot_error Plot vehicle position
        %
        % E.plot_error(OPTIONS) plot the error between actual and estimated vehicle 
        % path (x, y, theta) versus time.  Heading error is wrapped into the range [-pi,pi)
        %
        % Options::
        % 'bound',S    Display the confidence bounds (default 0.95).
        % 'color',C    Display the bounds using color C
        % LS           Use MATLAB linestyle LS for the plots
        %
        % Notes::
        % - The bounds show the instantaneous standard deviation associated
        %   with the state.  Observations tend to decrease the uncertainty
        %   while periods of dead-reckoning increase it.
        % - Set bound to zero to not draw confidence bounds.
        % - Ideally the error should lie "mostly" within the +/-3sigma
        %   bounds.
        %
        % See also EKF.plot_xy, EKF.plot_ellipse, EKF.plot_P.            
            opt.color = 'r';
            opt.confidence = 0.95;
            opt.nplots = 3;
            
            [opt,args] = tb_optparse(opt, varargin);
            
            clf
            if ekf.estVehicle
                err = zeros(length(ekf.history), 3);
                for i=1:length(ekf.history)
                    h = ekf.history(i);
                    % error is true - estimated
                    err(i,:) = ekf.robot.x_hist(i,:) - h.x_est(1:3)';
                    err(i,3) = angdiff(err(i,3));
                    P = diag(h.P);
                    pxy(i,:) = sqrt( chi2inv_rtb(opt.confidence, 2)*P(1:3) );
                end
                if nargout == 0
                    clf
                    t = 1:numrows(pxy);
                    t = [t t(end:-1:1)]';

                    subplot(opt.nplots*100+11)
                    if opt.confidence
                        edge = [pxy(:,1); -pxy(end:-1:1,1)];
                        h = patch(t, edge ,opt.color);
                        set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                    end
                    hold on
                    plot(err(:,1), args{:});
                    hold off
                    grid
                    ylabel('x error')
                    
                    subplot(opt.nplots*100+12)
                    if opt.confidence
                        edge = [pxy(:,2); -pxy(end:-1:1,2)];
                        h = patch(t, edge, opt.color);
                        set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                    end
                    hold on
                    plot(err(:,2), args{:});
                    hold off
                    grid
                    ylabel('y error')
                    
                    subplot(opt.nplots*100+13)
                    if opt.confidence
                        edge = [pxy(:,3); -pxy(end:-1:1,3)];
                        h = patch(t, edge, opt.color);
                        set(h, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                    end
                    hold on
                    plot(err(:,3), args{:});
                    hold off
                    grid
                    xlabel('Time step')
                    ylabel('\theta error')
                    
                    if opt.nplots > 3
                        subplot(opt.nplots*100+14);
                    end
                else
                    out = pxy;
                end
            end
        end
        
        function xy = get_map(ekf, varargin)
            %EKF.get_map Get landmarks
            %
            % P = E.get_map() is the estimated landmark coordinates (2xN) one per
            % column.  If the landmark was not estimated the corresponding column
            % contains NaNs.
            %
            % See also EKF.plot_map, EKF.plot_ellipse.
            
            xy = [];
            for i=1:numcols(ekf.landmarks)
                n = ekf.landmarks(1,i);
                if isnan(n)
                    % this landmark never observed
                    xy = [xy [NaN; NaN]];
                    continue;
                end
                % n is an index into the *landmark* part of the state
                % vector, we need to offset it to account for the vehicle
                % state if we are estimating vehicle as well
                if ekf.estVehicle
                    n = n + 3;
                end
                xf = ekf.x_est(n:n+1);
                xy = [xy xf];
            end
        end
        
        function plot_map(ekf, varargin)
        %EKF.plot_map Plot landmarks
        %
        % E.plot_map(OPTIONS) overlay the current plot with the estimated landmark 
        % position (a +-marker) and a covariance ellipses.
        %
        % E.plot_map(LS, OPTIONS) as above but pass line style arguments
        % LS to plot_ellipse.
        %
        % Options::
        % 'confidence',C   Draw ellipse for confidence value C (default 0.95)
        %
        % See also EKF.get_map, EKF.plot_ellipse.

        % TODO:  some option to plot map evolution, layered ellipses

            opt.confidence = 0.95;
            
            [opt,args] = tb_optparse(opt, varargin);
            
            xy = [];
            for i=1:numcols(ekf.landmarks)
                n = ekf.landmarks(1,i);
                if isnan(n)
                    % this landmark never observed
                    xy = [xy [NaN; NaN]];
                    continue;
                end
                % n is an index into the *landmark* part of the state
                % vector, we need to offset it to account for the vehicle
                % state if we are estimating vehicle as well
                if ekf.estVehicle
                    n = n + 3;
                end
                xf = ekf.x_est(n:n+1);
                P = ekf.P_est(n:n+1,n:n+1);
                % TODO reinstate the interval landmark
                %plot_ellipse(xf, P, interval, 0, [], varargin{:});
                plot_ellipse( P * chi2inv_rtb(opt.confidence, 2), xf, args{:});
                plot(xf(1), xf(2), 'k.', 'MarkerSize', 10)
            end
        end

        function P = get_P(ekf, k)
            %EKF.get_P Get covariance magnitude
            %
            % E.get_P() is a vector of estimated covariance magnitude at each time step.
            
            P = zeros(length(ekf.history),1);
            for i=1:length(ekf.history)
                P(i) = sqrt(det(ekf.history(i).P));
            end
        end
        
        function plot_P(ekf, varargin)
            %EKF.plot_P Plot covariance magnitude
            %
            % E.plot_P() plots the estimated covariance magnitude against
            % time step.
            %
            % E.plot_P(LS) as above but the optional line style arguments
            % LS are passed to plot.
            
            p = ekf.get_P();
            
            plot(p, varargin{:});
            xlabel('Time step');
            ylabel('(det P)^{0.5}')
            
        end

        function show_P(ekf, k)
            clf
            if nargin < 2
                k = length(ekf.history);
            end
            z = log10(abs(ekf.history(k).P));
            mn = min(z(~isinf(z)))
            z(isinf(z)) = mn;
            cmap = flip( gray(256), 1);
            %colormap(parula);
            colormap(flipud(bone))
            c = gray;
            c = [ones(numrows(c),1) 1-c(:,1:2)];
            colormap(c)
            
%             imshow(z, ...
%                 'DisplayRange', [min(z(:)) max(z(:))], ...
%                 'ColorMap', cmap, ...
%                 'InitialMagnification', 'fit'   )
            image(z, 'CDataMapping', 'scaled')
            xlabel('state'); ylabel('state');

            c = colorbar();
            c.Label.String = 'log covariance';
        end
        
        
        function plot_ellipse(ekf, varargin)
            %EKF.plot_ellipse Plot vehicle covariance as an ellipse
            %
            % E.plot_ellipse() overlay the current plot with the estimated
            % vehicle position covariance ellipses for 20 points along the
            % path.
            %
            % E.plot_ellipse(LS) as above but pass line style arguments
            % LS to plot_ellipse.
            %
            % Options::
            % 'interval',I        Plot an ellipse every I steps (default 20)
            % 'confidence',C      Confidence interval (default 0.95)
            %
            % See also plot_ellipse.

            opt.interval = round(length(ekf.history)/20);
            opt.confidence = 0.95;
            
            [opt,args] = tb_optparse(opt, varargin);
            
            holdon = ishold;
            hold on
            for i=1:opt.interval:length(ekf.history)
                h = ekf.history(i);
                %plot_ellipse(h.x_est(1:2), h.P(1:2,1:2), 1, 0, [], varargin{:});
                plot_ellipse(h.P(1:2,1:2) * chi2inv_rtb(opt.confidence, 2), h.x_est(1:2), args{:});
            end
            if ~holdon
                hold off
            end
        end
            
        function display(ekf)
            %EKF.display Display status of EKF object
            %
            % E.display() displays the state of the EKF object in
            % human-readable form.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a EKF object and the command has no trailing
            %   semicolon.
            %
            % See also EKF.char.
            
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(ekf) );
        end % display()

        function s = char(ekf)
            %EKF.char Convert to string
            %
            % E.char() is a string representing the state of the EKF
            % object in human-readable form.
            %
            % See also EKF.display.
            s = sprintf('EKF object: %d states', length(ekf.x_est));
            e = '';
            if ekf.estVehicle
                e = [e 'Vehicle '];
            end
            if ekf.estMap
                e = [e 'Map '];
            end
            s = char(s, ['  estimating: ' e]);
            if ~isempty(ekf.robot)
                s = char(s, char(ekf.robot));
            end
            if ~isempty(ekf.sensor)
                s = char(s, char(ekf.sensor));
            end
            s = char(s, ['W_est:  ' mat2str(ekf.W_est, 3)] );
            s = char(s, ['V_est:  ' mat2str(ekf.V_est, 3)] );
        end


    end % method

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %    P R I V A T E    M E T H O D S
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Access=protected)
        function x_est = step(ekf, opt)

            %fprintf('-------step\n');
            % move the robot along its path and get odometry
            odo = ekf.robot.step();

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% do the prediction
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            if ekf.estVehicle
                % split the state vector and covariance into chunks for 
                % vehicle and map
                xv_est = ekf.x_est(1:3);
                xm_est = ekf.x_est(4:end);

                Pvv_est = ekf.P_est(1:3,1:3);
                Pmm_est = ekf.P_est(4:end,4:end);
                Pvm_est = ekf.P_est(1:3,4:end);
            else
                xm_est = ekf.x_est;
                %Pvv_est = ekf.P_est;
                Pmm_est = ekf.P_est;
            end

            if ekf.estVehicle
                % evaluate the state update function and the Jacobians
                % if vehicle has uncertainty, predict its covariance
                xv_pred = ekf.robot.f(xv_est', odo)';

                Fx = ekf.robot.Fx(xv_est, odo);
                Fv = ekf.robot.Fv(xv_est, odo);
                Pvv_pred = Fx*Pvv_est*Fx' + Fv*ekf.V_est*Fv';
             else
                % otherwise we just take the true robot state
                xv_pred = ekf.robot.x;
             end

            if ekf.estMap
                if ekf.estVehicle
                    % SLAM case, compute the correlations
                    Pvm_pred = Fx*Pvm_est;
                end
                Pmm_pred = Pmm_est;
                xm_pred = xm_est;
            end

            % put the chunks back together again
            if ekf.estVehicle && ~ekf.estMap
                % vehicle only
                x_pred = xv_pred;
                P_pred =  Pvv_pred;
            elseif ~ekf.estVehicle && ekf.estMap
                % map only
                x_pred = xm_pred;
                P_pred = Pmm_pred;
            elseif ekf.estVehicle && ekf.estMap
                % vehicle and map
                x_pred = [xv_pred; xm_pred];
                P_pred = [ Pvv_pred Pvm_pred; Pvm_pred' Pmm_pred];
            end

            % at this point we have:
            %   xv_pred the state of the vehicle to use to 
            %           predict observations
            %   xm_pred the state of the map
            %   x_pred  the full predicted state vector
            %   P_pred  the full predicted covariance matrix

            % initialize the variables that might be computed during
            % the update phase

            doUpdatePhase = false;

            %fprintf('x_pred:'); x_pred'

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% process observations
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            sensorReading = false;
            if ~isempty(ekf.sensor)
                % read the sensor
                [z,js] = ekf.sensor.reading();
                                
                % test if the sensor has returned a reading at this time interval
                sensorReading = js > 0;
            end

            if sensorReading
                % here for MBL, MM, SLAM

                % compute the innovation
                z_pred = ekf.sensor.h(xv_pred', js)';
                innov(1) = z(1) - z_pred(1);
                innov(2) = angdiff(z(2), z_pred(2));

                if ekf.estMap
                    % the map is estimated MM or SLAM case
                    if ekf.seenBefore(js)

                        % get previous estimate of its state
                        jx = ekf.landmarks(1,js);
                        xf = xm_pred(jx:jx+1);

                        % compute Jacobian for this particular landmark
                        Hx_k = ekf.sensor.Hp(xv_pred', xf);
                        % create the Jacobian for all landmarks
                        Hx = zeros(2, length(xm_pred));
                        Hx(:,jx:jx+1) = Hx_k;

                        Hw = ekf.sensor.Hw(xv_pred, xf);

                        if ekf.estVehicle
                            % concatenate Hx for for vehicle and map
                            Hxv = ekf.sensor.Hx(xv_pred', xf);
                            Hx = [Hxv Hx];
                        end
                        doUpdatePhase = true;

        %                if mod(i, 40) == 0
        %                    plot_ellipse(x_est(jx:jx+1), P_est(jx:jx+1,jx:jx+1), 5);
        %                end
                    else
                        % get the extended state
                        [x_pred, P_pred] = ekf.extendMap(P_pred, xv_pred, xm_pred, z, js);
                        doUpdatePhase = false;
                    end
                else
                    % the map is given, MBL case
                    Hx = ekf.sensor.Hx(xv_pred', js);
                    Hw = ekf.sensor.Hw(xv_pred', js);
                    doUpdatePhase = true;
                end
            end

                % doUpdatePhase flag indicates whether or not to do
                % the update phase of the filter
                %
                %  DR                        always false
                %  map-based localization    if sensor reading
                %  map creation              if sensor reading & not first
                %                              sighting
                %  SLAM                      if sighting of a previously
                %                              seen landmark

            if doUpdatePhase
                %fprintf('do update\n');
                %% we have innovation, update state and covariance
                % compute x_est and P_est

                % compute innovation covariance
                S = Hx*P_pred*Hx' + Hw*ekf.W_est*Hw';

                % compute the Kalman gain
                K = P_pred*Hx' / S;

                % update the state vector
                x_est = x_pred + K*innov';
            
                if ekf.estVehicle
                    % wrap heading state for a vehicle
                    x_est(3) = angdiff(x_est(3));
                end
            
                % update the covariance
                if ekf.joseph
                    % we use the Joseph form
                    I = eye(size(P_pred));
                    P_est = (I-K*Hx)*P_pred*(I-K*Hx)' + K*ekf.W_est*K';
                else
                    P_est = P_pred - K*S*K';
                end

                % enforce P to be symmetric
                P_est = 0.5*(P_est+P_est');
            else
                % no update phase, estimate is same as prediction
                x_est = x_pred;
                P_est = P_pred;
                innov = [];
                S = [];
                K = [];
            end

            %fprintf('X:'); x_est'

            % update the state and covariance for next time
            ekf.x_est = x_est;
            ekf.P_est = P_est;
            
            
            % record time history
            if ekf.keepHistory
                hist = [];
                hist.x_est = x_est;
                hist.odo = odo;
                hist.P = P_est;
                hist.innov = innov;
                hist.S = S;
                hist.K = K;
                ekf.history = [ekf.history hist];
            end
        end

        function s = seenBefore(ekf, jf)
            if ~isnan(ekf.landmarks(1,jf))
                %% we have seen this landmark before, update number of sightings
                if ekf.verbose
                    fprintf('landmark %d seen %d times before, state_idx=%d\n', ...
                        jf, ekf.landmarks(2,jf), ekf.landmarks(1,jf));
                end
                ekf.landmarks(2,jf) = ekf.landmarks(2,jf)+1;  
                s = true;
            else
                s = false;
            end
        end
                

        function [x_ext, P_ext] = extendMap(ekf, P, xv, xm, z, jf)
            
            %% this is a new landmark, we haven't seen it before
            % estimate position of landmark in the world based on 
            % noisy sensor reading and current vehicle pose

            if ekf.verbose
                fprintf('landmark %d first sighted\n', jf);
            end

            % estimate its position based on observation and vehicle state
            xf = ekf.sensor.g(xv, z);

            % append this estimate to the state vector
            if ekf.estVehicle
                x_ext = [xv; xm; xf];
            else
                x_ext = [xm; xf];
            end

            % get the Jacobian for the new landmark
            Gz = ekf.sensor.Gz(xv, z);

            % extend the covariance matrix
            if ekf.estVehicle
                Gx = ekf.sensor.Gx(xv, z);
                n = length(ekf.x_est);
                M = [eye(n) zeros(n,2); Gx zeros(2,n-3) Gz];
                P_ext = M*blkdiag(P, ekf.W_est)*M';
            else
                P_ext = blkdiag(P, Gz*ekf.W_est*Gz');
            end
    
            % record the position in the state vector where this 
            % landmark's state starts
            ekf.landmarks(1,jf) = length(xm)+1;
            %ekf.landmarks(1,jf) = length(ekf.x_est)-1;
            ekf.landmarks(2,jf) = 1;        % seen it once

            if ekf.verbose
                fprintf('extended state vector\n');
            end

            % plot an ellipse at this time
%                jx = landmarks(1,jf);
%                plot_ellipse(x_est(jx:jx+1), P_est(jx:jx+1,jx:jx+1), 5);

        end
    end % private methods
end % classdef

function f = chi2inv_rtb(confidence, n)
 
 assert(n == 2, 'chi2inv_rtb: only valid for 2DOF');
 
 c = linspace(0,1,101);
 % build a lookup table:
 %x = chi2inv(c,2)
 %fprintf('%f ');
 x = [0.000000 0.020101 0.040405 0.060918 0.081644 0.102587 0.123751 0.145141 0.166763 0.188621 0.210721 0.233068 0.255667 0.278524 0.301646 0.325038 0.348707 0.372659 0.396902 0.421442 0.446287 0.471445 0.496923 0.522730 0.548874 0.575364 0.602210 0.629421 0.657008 0.684981 0.713350 0.742127 0.771325 0.800955 0.831031 0.861566 0.892574 0.924071 0.956072 0.988593 1.021651 1.055265 1.089454 1.124238 1.159637 1.195674 1.232372 1.269757 1.307853 1.346689 1.386294 1.426700 1.467938 1.510045 1.553058 1.597015 1.641961 1.687940 1.735001 1.783196 1.832581 1.883217 1.935168 1.988505 2.043302 2.099644 2.157619 2.217325 2.278869 2.342366 2.407946 2.475749 2.545931 2.618667 2.694147 2.772589 2.854233 2.939352 3.028255 3.121295 3.218876 3.321462 3.429597 3.543914 3.665163 3.794240 3.932226 4.080442 4.240527 4.414550 4.605170 4.815891 5.051457 5.318520 5.626821 5.991465 6.437752 7.013116 7.824046 9.210340 Inf];
 
 f = interp1(c, x, confidence);

end
 
