%ParticleFilter Particle filter class
%
% Monte-carlo based localisation for estimating vehicle position based on
% odometry and observations of known landmarks.
%
%
% Methods::
% run        run the particle filter
% plot_xy    display estimated vehicle path
% plot_pdf   display particle distribution
%
% Properties::
%  robot        reference to the robot object
%  sensor       reference to the sensor object
%  history      vector of structs that hold the detailed information from
%               each time step
%  nparticles   number of particles used
%  x            particle states; nparticles x 3
%  weight       particle weights; nparticles x 1
%  x_est        mean of the particle population
%  std          standard deviation of the particle population
%  Q            covariance of noise added to state at each step
%  L            covariance of likelihood model
%  dim          maximum xy dimension
%
% Example::
%
% Create a landmark map
%    map = Map(20);
% and a vehicle with odometry covariance and a driver
%    W = diag([0.1, 1*pi/180].^2);
%    veh = Vehicle(W);
%    veh.add_driver( RandomPath(10) );
% and create a range bearing sensor
%    R = diag([0.005, 0.5*pi/180].^2);
%    sensor = RangeBearingSensor(veh, map, R);
%
% For the particle filter we need to define two covariance matrices.  The
% first is is the covariance of the random noise added to the particle
% states at each iteration to represent uncertainty in configuration.   
%    Q = diag([0.1, 0.1, 1*pi/180]).^2;
% and the covariance of the likelihood function applied to innovation
%    L = diag([0.1 0.1]);
% Now construct the particle filter
%    pf = ParticleFilter(veh, sensor, Q, L, 1000);
% which is configured with 1000 particles.  The particles are initially
% uniformly distributed over the 3-dimensional configuration space.
%    
% We run the simulation for 1000 time steps
%    pf.run(1000);
% then plot the map and the true vehicle path
%    map.plot();
%    veh.plot_xy('b');
% and overlay the mean of the particle cloud
%    pf.plot_xy('r');
% We can plot the standard deviation against time
%    plot(pf.std(1:100,:))
% The particles are a sampled approximation to the PDF and we can display
% this as
%    pf.plot_pdf()
%
% Acknowledgement::
% 
% Based on code by Paul Newman, Oxford University, 
% http://www.robots.ox.ac.uk/~pnewman
%
% Reference::
%
%   Robotics, Vision & Control,
%   Peter Corke,
%   Springer 2011
%
% See also Vehicle, RandomPath, RangeBearingSensor, Map, EKF.

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


%note this is not coded efficiently but rather to make the ideas clear
%all loops should be vectorized but that gets a little matlab-speak intensive
%and may obliterate the elegance of a particle filter....

classdef ParticleFilter < handle
%TODO
% x_est should be a weighted mean
% std should be a weighted std (x-mean)' W (x-mean)  ???
    properties
        robot
        sensor
        nparticles
        x           % particle states; nparticles x 3
        weight      % particle weights; nparticles x 1
        x_est        % mean of the particle population
        std         % standard deviation of the particle population
        Q           % covariance of noise added to state at each step
        L           % covariance of likelihood model
        history
        dim         % maximum xy dimension

        h           % graphics handle for particles
    end % properties

    methods
        function pf = ParticleFilter(robot, sensor, Q, L, np)
            %ParticleFilter.ParticleFilter Particle filter constructor
            %
            % PF = ParticleFilter(VEHICLE, SENSOR, Q, L, NP) is a particle
            % filter that estimates the state of the VEHICLE with a sensor
            % SENSOR.  Q is covariance of the noise added to the particles
            % at each step (diffusion), L is the covariance used in the
            % sensor likelihood model, and NP is the number of particles.
            %
            %
            % Notes::
            % - ParticleFilter subclasses Handle, so it is a reference object.
            % - the initial particle distribution is uniform over the map,
            %   essentially the kidnapped robot problem which is unrealistic
            %
            % See also Vehicle, Sensor, RangeBearingSensor, Map.

            pf.robot = robot;
            pf.sensor = sensor;
            pf.Q = Q;
            pf.L = L;
            pf.nparticles = np;

            % create initial particle distribution as uniformly randomly distributed
            % over the map area and heading angles
            pf.dim = sensor.map.dim;
            pf.history = [];
            pf.x = [];
            pf.weight = [];
        end


        function init(pf)
            pf.robot.init();
            pf.history = [];
            pf.x = (2*rand([pf.nparticles,3]) - 1) * diag([pf.dim, pf.dim, pi]);
            pf.weight = ones(pf.nparticles, 1);
            pf.x_est = [];
            pf.std = [];
        end

        function run(pf, niter)
            %ParticleFilter.run Run the particle filter
            %
            % PF.run(N) run the filter for N time steps.
            %
            % Notes::
            % - all previously estimated states and estimation history is
            %   cleared.
            pf.init();
            pf.sensor.map.plot();
            a = axis;
            a(5:6) = [-pi pi];
            axis(a)
            zlabel('heading (rad)');

            % display the initial particles
            pf.h = plot3(pf.x(:,1), pf.x(:,2), pf.x(:,3), 'g.');

            h = pf.robot.plot();

            % iterate over time
            for i=1:niter
                pf.step();
            end
         end

         function step(pf)
             
            %fprintf('---- step\n');
            odo = pf.robot.step();        % move the robot

            % update the particles based on odometry
            pf.predict(odo);

            % get a sensor reading
            [z,jf] = pf.sensor.reading();         

            if ~isnan(jf)
                pf.observe(z, jf);
                %fprintf(' observe beacon %d\n', jf);

                pf.select();
            end
            % our estimate is simply the mean of the particles
            mn = mean(pf.x);
            pf.x_est = [pf.x_est; mn];
            s = std(pf.x);

            % std is more complex for angles, need to account for 2pi wrap
            s(3) = sqrt(sum(angdiff(pf.x(:,3), mn(3)).^2)) / (pf.nparticles-1);
            pf.std = [pf.std; s];

            % display the updated particles
            set(pf.h, 'Xdata', pf.x(:,1), 'Ydata', pf.x(:,2), 'Zdata', pf.x(:,3));

            pf.robot.plot();
            drawnow

            hist = [];
            hist.x_est = pf.x;
            hist.w = pf.weight;
            pf.history = [pf.history hist];
        end

        % step 2
        % update the particle state based on odometry and a random perturbation
        function predict(pf, odo)
            for i=1:pf.nparticles
                x = pf.robot.f( pf.x(i,:), odo) + sqrt(pf.Q)*randn(3,1);   
                x(3) = angdiff(x(3));
                pf.x(i,:) = x';
            end
        end

        % step 3
        % predict observation and score the particles
        function observe(pf, z, jf)
        
            for p = 1:pf.nparticles
                % what do we expect observation to be for this particle?
                % use the sensor model h(.)
                z_pred = pf.sensor.h( pf.x(p,:), jf);
                
                % how different is it
                innov = zeros(2,1);
                innov(1) = z(1) - z_pred(1);
                innov(2) = angdiff(z(2), z_pred(2));
                
                % get likelihood (new importance). Assume Gaussian but any PDF works!
                % If predicted obs is very different from actual obs this score will be low
                %  ie. this particle is not very good at predicting the observation.
                % A lower score means it is less likely to be selected for the next generation...
                % The weight is never zero.
                pf.weight(p) = exp(-0.5*innov'*inv(pf.L)*innov) + 0.05;
            end  
        end

        % step 4
        % select particles based on their weights
        function select(pf, odo)
            
            % particles with large weights will occupy a greater percentage of the
            % y axis in a cummulative plot
            CDF = cumsum(pf.weight)/sum(pf.weight);

            % so randomly (uniform) choosing y values is more likely to correspond to
            % better particles...
            iSelect  = rand(pf.nparticles,1);

            % find the particle that corresponds to each y value (just a look up)
            iNextGeneration = interp1(CDF, 1:pf.nparticles, iSelect, 'nearest', 'extrap');

            % copy selected particles for next generation..
            pf.x = pf.x(iNextGeneration,:);
        end

        function plot_xy(pf, varargin)
            %ParticleFilter.plot_xy Plot vehicle position
            %
            % PF.plot_xy() plot the estimated vehicle path in the xy-plane.
            %
            % PF.plot_xy(LS) as above but the optional line style arguments
            % LS are passed to plot.
            plot(pf.x_est(:,1), pf.x_est(:,2), varargin{:});
        end


        function plot_pdf(pf)
            %ParticleFilter.plot_pdf Plot particles as a PDF
            %
            % PF.plot_pdf() plots a sparse PDF as a series of vertical line
            % segments of height equal to particle weight.
            clf
            hold on
            for p = 1:pf.nparticles
                x = pf.x(p,:);
                plot3([x(1) x(1)], [x(2) x(2)], [0 pf.weight(p)]);
            end
        end
    end % methods
end
