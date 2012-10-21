%Navigation Navigation superclass
%
% An abstract superclass for implementing navigation classes.  
%
% Methods::
%   plot        Display the occupancy grid
%   visualize   Display the occupancy grid (deprecated)
%   plan        Plan a path to goal
%   path        Return/animate a path from start to goal
%   display     Display the parameters in human readable form
%   char        Convert to string
%
%   rand        Uniformly distributed random number
%   randn       Normally distributed random number
%   randi       Uniformly distributed random integer
%
% Properties (read only)::
%   occgrid   Occupancy grid representing the navigation environment
%   goal      Goal coordinate
%   seed0     Random number state
%
% Methods that must be provided in subclass::
%   plan      Generate a plan for motion to goal
%   next      Returns coordinate of next point along path
%
% Methods that may be overriden in a subclass::
%   goal_set        The goal has been changed by nav.goal = (a,b)
%   navigate_init   Start of path planning.
% 
% Notes::
% - Subclasses the MATLAB handle class which means that pass by reference semantics
%   apply.
% - A grid world is assumed and vehicle position is quantized to grid cells.
% - Vehicle orientation is not considered.
% - The initial random number state is captured as seed0 to allow rerunning an
%   experiment with an interesting outcome.
%
% See also Dstar, Dxform, PRM, RRT.

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

% Peter Corke 8/2009.

% TODO
%  keep dimensions of workspace in this object, have a setaxes() method
%  which transfers the dimensions to the current axes.

classdef Navigation < handle

    properties
        occgrid     % occupancy grid
        goal        % goal coordinate

        navhook     % function handle, called on each navigation iteration
        verbose     % verbosity
        seed            % current random seed
        spincount

        randstream
        seed0
    end


    % next() should be protected and abstract, but this doesnt work
    % properly
    methods (Abstract)
        plan(obj)
        n = next(obj)
    end % method Abstract

    methods

        % TODO fix up set methods for goal
        % setup argument callback like features, can we inherit from that.
        % occ grid should be an option

        % constructor

        function nav = Navigation(varargin)
        %Navigation.Navigation Create a Navigation object
        %
        % N = Navigation(OCCGRID, OPTIONS) is a Navigation object that holds an
        % occupancy grid OCCGRID.  A number of options can be be passed.
        %
        % Options::
        % 'navhook',F   Specify a function to be called at every step of path
        % 'goal',G      Specify the goal point (2x1)
        % 'verbose'     Display debugging information
        % 'inflate',K   Inflate all obstacles by K cells.
        % 'private'     Use private random number stream.
        % 'reset'       Reset random number stream.
        % 'seed',S      Set the initial state of the random number stream.  S must
        %               be a proper random number generator state such as saved in
        %               the seed0 property of an earlier run.
        %
        % Notes::
        % - In the occupancy grid a value of zero means free space and non-zero means
        %   occupied (not driveable).
        % - Obstacle inflation is performed with a round structuring element (kcircle).
        % - The 'private' option creates a private random number stream for the methods 
        %   rand, randn and randi.  If not given the global stream is used.

            if nargin >= 1 && isnumeric(varargin{1})
                nav.occgrid = varargin{1};
                varargin = varargin(2:end);
            end
            
            % default values of options
            opt.goal = [];
            opt.inflate = 0;
            opt.navhook = [];
            opt.private = false;
            opt.reset = false;
            opt.seed = [];
            
            [opt,args] = tb_optparse(opt, varargin);

            % optionally inflate the obstacles
            if opt.inflate > 0
                nav.occgrid = idilate(nav.occgrid, kcircle(opt.inflate));
            end
            
            % copy other options into the object
            nav.verbose = opt.verbose;
            nav.navhook = opt.navhook;
            if ~isempty(opt.goal)
                nav.goal = opt.goal(:)';
            end

            % create a private random number stream if required
            if opt.private
                nav.randstream = RandStream.create('mt19937ar');
            else
                nav.randstream = RandStream.getGlobalStream();
            end

            % reset the random number stream if required
            if opt.reset
                nav.randstream.reset();
            end

            % return the random number stream to known state if required
            if ~isempty(opt.seed)
                set(nav.randstream.set(opt.seed));
            end

            % save the current state in case it later turns out to give interesting results
            nav.seed0 = nav.randstream.State;

            nav.spincount = 0;
        end

        function r = rand(nav, varargin)
        %Navigation.rand Uniformly distributed random number
        %
        % R = N.rand() return a uniformly distributed random number from
        % a private random number stream.
        %
        % R = N.rand(M) as above but return a matrix (MxM) of random numbers.
        %
        % R = N.rand(L,M) as above but return a matrix (LxM) of random numbers.
        %
        % Notes::
        % - Accepts the same arguments as rand().
        % - Seed is provided to Navigation constructor.
        %
        % See also rand, RandStream.
            r = nav.randstream.rand(varargin{:});
        end

        function r = randn(nav, varargin)
        %Navigation.randn Normally distributed random number
        %
        % R = N.randn() return a normally distributed random number from
        % a private random number stream.
        %
        % R = N.randn(M) as above but return a matrix (MxM) of random numbers.
        %
        % R = N.randn(L,M) as above but return a matrix (LxM) of random numbers.
        %
        %
        % Notes::
        % - Accepts the same arguments as randn().
        % - Seed is provided to Navigation constructor.
        %
        % See also randn, RandStream.
            r = nav.randstream.randn(varargin{:});
        end

        function r = randi(nav, varargin)
        %Navigation.randi Integer random number
        %
        % I = N.randi(RM) return a uniformly distributed random integer in the 
        % range 1 to RM from a private random number stream.
        %
        % I = N.randi(RM, M) as above but return a matrix (MxM) of random integers.
        %
        % I = N.randn(RM, L,M) as above but return a matrix (LxM) of random integers.
        %
        %
        % Notes::
        % - Accepts the same arguments as randn().
        % - Seed is provided to Navigation constructor.
        %
        % See also randn, RandStream.
            r = nav.randstream.randi(varargin{:});
        end

        % invoked whenever the goal is set
        function set.goal(nav, goal)

            if ~isempty(nav.occgrid) && nav.occgrid( goal(2), goal(1)) == 1
                error('Navigation: cant set goal inside obstacle');
            end
            
            goal = goal(:);
            if ~(all(size(goal) == size(nav.goal)) && all(goal == nav.goal))
                % goal has changed
                nav.goal = goal(:);
                nav.goal_change();
            end
        end
        
        function goal_change(nav)
            %Navigation.goal_change Notify change of goal
            %
            % Invoked when the goal property of the object is changed.  Typically this
            % is overriden in a subclass to take particular action such as invalidating
            % a costmap.
        end
        


        function pp = path(nav, start)
            %Navigation.path Follow path from start to goal
            %
            % N.path(START) animates the robot moving from START (2x1) to the goal (which is a 
            % property of the object).
            %
            % N.path() as above but first displays the occupancy grid, and prompts the user to 
            % click a start location.
            % the object).
            %
            % X = N.path(START) returns the path (2xM) from START to the goal (which is a property of 
            % the object).
            %
            % The method performs the following steps:
            %
            %  - Get start position interactively if not given
            %  - Initialized navigation, invoke method N.navigate_init()
            %  - Visualize the environment, invoke method N.plot()
            %  - Iterate on the next() method of the subclass
            %
            % See also Navigation.plot, Navigation.goal.

            % if no start point given, display the map, and prompt the user to select
            % a start point
            if nargin < 2
                % display the world
                nav.plot();

                % prompt the user to click a goal point
                fprintf('** click a starting point ');
                [x,y] = ginput(1);
                fprintf('\n');
                start = round([x;y]);
            end
            start = start(:);

            % if no output arguments given, then display the world
            if nargout == 0
                % render the world
                nav.plot();
                hold on
            end
            
            nav.navigate_init(start);

            p = [];
            % robot is a column vector
            robot = start;

            % iterate using the next() method until we reach the goal
            while true
                if nargout == 0
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
                    drawnow 
                end

                % move to next point on path
                robot = nav.next(robot);

                % are we there yet?
                if isempty(robot)
                    % yes, exit the loop
                    break
                else
                    % no, append it to the path
                    p = [p; robot(:)'];
                end

                % invoke the navhook function
                if isa(nav.navhook, 'function_handle')
                    nav.navhook(nav, robot(1), robot(2));
                end
            end

            % only return the path if required
            if nargout > 0
                pp = p;
            end
        end

        function visualize(nav, varargin)
            warning('visualize method deprecated for Navigation classes, use plot instead');
            nav.plot(varargin{:});
        end

        function plot(nav, varargin)
        %Navigation.plot  Visualize navigation environment
        %
        % N.plot() displays the occupancy grid in a new figure.
        %
        % N.plot(P) as above but overlays the points along the path (Mx2) matrix.
        %
        % Options::
        %  'goal'         Superimpose the goal position if set
        %  'distance',D   Display a distance field D behind the obstacle map.  D is
        %                 a matrix of the same size as the occupancy grid.            
            
            opt.goal = false;
            opt.distance = [];
            
            [opt,args] = tb_optparse(opt, varargin);
            
            clf
            if isempty(opt.distance)
                % create color map for free space + obstacle:
                %   free space, color index = 1, white, 
                %   obstacle, color index = 2, red
                cmap = [1 1 1; 1 0 0];  % non obstacles are white
                image(nav.occgrid+1, 'CDataMapping', 'direct');
                colormap(cmap)
                
            else
                % create color map for distance field + obstacle:
                %   obstacle, color index = 1, red
                %   free space, color index > 1, greyscale 
                
                % find maximum distance, ignore infinite values in
                % obstacles
                d = opt.distance(isfinite(opt.distance));
                maxdist = max(d(:)) + 1;
                
                % create the color map
                cmap = [1 0 0; gray(maxdist)];
                
                % ensure obstacles appear as red pixels
                opt.distance(nav.occgrid > 0) = 0;
                
                % display it with colorbar
                image(opt.distance+1, 'CDataMapping', 'direct');
                set(gcf, 'Renderer', 'Zbuffer')
                colormap(cmap)
                colorbar
            end
            
            % label the grid
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            grid on
            hold on
            
            if ~isempty(args)
                p = args{1};
                if numcols(p) ~= 2
                    error('expecting Nx2 matrix of points');
                end
                plot(p(:,1), p(:,2), 'g.', 'MarkerSize', 12);
            end
            
            if ~isempty(nav.goal) && opt.goal
                plot(nav.goal(1), nav.goal(2), 'bd', 'MarkerFaceColor', 'b');
            end
            hold off
        end

        function navigate_init(nav, start)
            %Navigation.navigate_init Notify start of path
            %
            % Invoked when the path() method is invoked. Typically overriden in a subclass 
            % to take particular action such as computing some path parameters.
            % start is the initial position for this path, and nav.goal is the final position.
        end


        function display(nav)
        %Navigation.display Display status of navigation object
        %
        % N.display() displays the state of the navigation object in 
        % human-readable form.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a Navigation object and the command has no trailing
        %   semicolon.
        %
        % See also Navigation.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( nav.char() );
        end % display()

        function s = char(nav)
        %Navigation.char Convert to string
        %
        % N.char() is a string representing the state of the navigation 
        % object in human-readable form.
            s = [class(nav) ' navigation class:'];
            s = char(s, sprintf('  occupancy grid: %dx%d', size(nav.occgrid)));
            if ~isempty(nav.goal)
                if length(nav.goal) == 2
                    s = char(s, sprintf('  goal: (%d,%d)', nav.goal) );
                else
                    s = char(s, sprintf('  goal: (%g,%g, %g)', nav.goal) );
                    
                end
            end
        end

        
        function verbosity(nav, v)
        %Navigation.verbosity Set verbosity
        %
        % N.verbosity(V) set verbosity to V, where 0 is silent and greater
        % values display more information.
            nav.verbose = v;
        end
            
        % called at each point on the path as
        %   navhook(nav, robot)
        %
        % can be used for logging data, animation, etc.
        function navhook_set(nav, navhook)
            nav.navhook = navhook;
        end
        
        function message(nav, varargin)
        %Navigation.message Display debug message
        %
        % N.message(S) displays the string S if the verbose property is true.
        %
        % N.message(FMT, ARGS) as above but accepts printf() like semantics.
            if nav.verbose
                fprintf([class(nav) ' debug:: ' sprintf(varargin{:}) '\n']);
            end
        end

        function spinner(nav)
        %Navigation.spinner Update progress spinner
        %
        % N.spinner() displays a simple ASCII progress spinner, a rotating bar.
            spinchars = '-\|/';
            nav.spincount = nav.spincount + 1;
            fprintf('\r%c', spinchars( mod(nav.spincount, length(spinchars))+1 ) );
        end

    end % method

end % classdef
