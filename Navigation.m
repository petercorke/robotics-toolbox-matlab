%Navigation Navigation superclass
%
% An abstract superclass for implementing planar grid-based navigation classes.  
%
% Methods::
%   Navigation        Superclass constructor
%   plan              Find a path to goal
%   query             Return/animate a path from start to goal
%   plot              Display the occupancy grid
%   display           Display the parameters in human readable form
%   char              Convert to string
%   isoccupied        Test if cell is occupied
%   rand              Uniformly distributed random number
%   randn             Normally distributed random number
%   randi             Uniformly distributed random integer
%--
%   progress_init     Create a progress bar
%   progress          Update progress bar
%   progress_delete   Remove progress bar
%
% Properties (read only)::
%   occgrid   Occupancy grid representing the navigation environment
%   goal      Goal coordinate
%   start     Start coordinate
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
% See also Bug2, Dstar, Dxform, PRM, Lattice, RRT.



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

% Peter Corke 8/2009.

% TODO
%  keep dimensions of workspace in this object, have a setaxes() method
%  which transfers the dimensions to the current axes.

classdef Navigation < handle

    properties
        options
        
        occgrid     % occupancy grid as provided by user
        occgridnav  % inflated occupancy grid
        goal        % goal coordinate
        start       % start coordinate

        verbose     % verbosity
        seed            % current random seed
        spincount

        randstream
        seed0
        
        w2g        % transform from world coordinates to grid coordinates
    end
    
    
    % we make this class abtract
    methods(Abstract)
        plan
        next
    end
    
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
        % 'goal',G        Specify the goal point (2x1)
        % 'inflate',K     Inflate all obstacles by K cells.
        % 'private'       Use private random number stream.
        % 'reset'         Reset random number stream.
        % 'verbose'       Display debugging information
        % 'seed',S        Set the initial state of the random number stream.  S must
        %                 be a proper random number generator state such as saved in
        %                 the seed0 property of an earlier run.
        %
        % Notes::
        % - In the occupancy grid a value of zero means free space and non-zero means
        %   occupied (not driveable).
        % - Obstacle inflation is performed with a round structuring element (kcircle) 
        %   with radius given by the 'inflate' option.
        % - Inflation requires either MVTB or IPT installed.
        % - The 'private' option creates a private random number stream for the methods 
        %   rand, randn and randi.  If not given the global stream is used.
        %
        % See also randstream.
        
        
        % TODO:
        %   - allow for an arbitrary transform from world coordinates to the grid
        %   - it needs to affect plot scaling, start and goal

            if nargin >= 1 && ( isnumeric(varargin{1}) || islogical(varargin{1}))
                % first argument is the map
                map = double( varargin{1} );
                varargin = varargin(2:end);
                if isnumeric(map) && ~isscalar(map)
                    nav.occgrid = map;
                    nav.w2g = SE2(0, 0, 0);
                elseif isstruct(map)
                     nav.occgrid = map.map;
                     nav.w2g = nav.T;
                end
            end
            
            % default values of options
            opt.goal = [];
            opt.inflate = 0;
            opt.private = false;
            opt.reset = false;
            opt.seed = [];
            opt.transform = SE2;
            
            [opt,lp.options] = tb_optparse(opt, varargin);

            % optionally inflate the obstacles

            if opt.inflate > 0
                if exist('idilate') == 2
                    % use MVTB
                    nav.occgridnav = idilate(nav.occgrid, kcircle(opt.inflate));
                elseif exist('imdilate') == 2
                    % use IPT
                    nav.occgridnav = imdilate(nav.occgrid, strel('disk',opt.inflate));
                else
                    error('RTB:Navigatio:Navigation', 'Need to have MVTB or IPT installed to perform obstacle inflation');
                end
            else
                nav.occgridnav = nav.occgrid;
            end
            
            % copy other options into the object
            nav.verbose = opt.verbose;
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
            
            nav.w2g = opt.transform;

            nav.spincount = 0;
        end

        function pp = query(nav, start, varargin)
            %Navigation.query Find a path from start to goal using plan
            %
            % N.query(START, OPTIONS) animates the robot moving from START (2x1) to the goal (which is a 
            % property of the object) using a previously computed plan.
            %
            % X = N.query(START, OPTIONS) returns the path (Mx2) from START to the goal (which is a property of 
            % the object).
            %
            % The method performs the following steps:
            %  - Initialize navigation, invoke method N.navigate_init()
            %  - Visualize the environment, invoke method N.plot()
            %  - Iterate on the next() method of the subclass until the goal is
            %    achieved.
            %
            % Options::
            % 'animate'    Show the computed path as a series of green dots.
            %
            % Notes::
            %  - If START given as [] then the user is prompted to click a point on the map.
            %
            %
            % See also Navigation.navigate_init, Navigation.plot, Navigation.goal.
            
            opt.animate = false;
            opt = tb_optparse(opt, varargin);
            
            % make sure start and goal are set and valid, optionally prompt
            nav.checkquery(start);
            
            if opt.animate
                nav.plot();
                hold on
            end
            
            % iterate using the next() method until we reach the goal
            robot = nav.start;
            path = nav.start(:);
            while true
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
                    drawnow
                end
                
                % move to next point on path
                robot = nav.next(robot);
                
                % are we there yet?
                if isempty(robot)
                    path = [path nav.goal(:)];
                    % yes, exit the loop
                    break
                else
                    path = [path robot(:)]; % append it to the path
                end
            end
            
            % return the path 
            if nargout > 0
                pp = path';
            end
        end

        function plot(nav, varargin)
        %Navigation.plot  Visualize navigation environment
        %
        % N.plot(OPTIONS) displays the occupancy grid in a new figure.
        %
        % N.plot(P, OPTIONS) as above but overlays the points along the path (2xM) matrix.
        %
        % Options::
        %  'distance',D    Display a distance field D behind the obstacle map.  D is
        %                  a matrix of the same size as the occupancy grid.
        %  'colormap',@f   Specify a colormap for the distance field as a function handle, eg. @hsv
        %  'beta',B        Brighten the distance field by factor B.
        %  'inflated'      Show the inflated occupancy grid rather than original
        %
        % Notes::
        % - The distance field at a point encodes its distance from the goal, small
        %   distance is dark, a large distance is bright.  Obstacles are encoded as
        %   red.
        % - Beta value -1<B<0 to darken, 0<B<+1 to lighten.
        %
        % See also Navigation.plot_fg, Navigation.plot_bg.
            nav.plot_bg(varargin{:});
            nav.plot_fg(varargin{:});
        end
        
        function plot_bg(nav, varargin)
        %Navigation.plot  Visualization background
        %
        % N.plot_bg(OPTIONS) displays the occupancy grid with occupied cells shown as
        % red and an optional distance field.
        %
        % N.plot_bg(P,OPTIONS) as above but overlays the points along the path (2xM) matrix. 
        %
        % Options::
        %  'distance',D      Display a distance field D behind the obstacle map.  D is
        %                    a matrix of the same size as the occupancy grid.
        %  'colormap',@f     Specify a colormap for the distance field as a function handle, eg. @hsv
        %  'beta',B          Brighten the distance field by factor B.
        %  'inflated'        Show the inflated occupancy grid rather than original
        %  'pathmarker',M    Options to draw a path point
        %  'startmarker',M   Options to draw the start marker
        %  'goalmarker',M    Options to draw the goal marker
        %
        % Notes::
        % - The distance field at a point encodes its distance from the goal, small
        %   distance is dark, a large distance is bright.  Obstacles are encoded as
        %   red.
        % - Beta value -1<B<0 to darken, 0<B<+1 to lighten.
        %
        % See also Navigation.plot, Navigation.plot_fg, brighten.
            
            opt.distance = [];
            opt.colormap = @bone;
            opt.beta = 0.2;
            opt.inflated = false;

            opt = tb_optparse(opt, varargin);
            
            if opt.inflated
                occgrid = nav.occgridnav;
            else
                occgrid = nav.occgrid;
            end
            
            clf
            if isempty(opt.distance) || all(all(~isfinite(opt.distance)))
                % create color map for free space + obstacle:
                %   free space, color index = 1, white, 
                %   obstacle, color index = 2, red
                cmap = [1 1 1; 1 0 0];  % non obstacles are white
                image(occgrid+1, 'CDataMapping', 'direct', ...
                    'AlphaData', occgrid);
                colormap(cmap)
                
            else
                % create color map for distance field + obstacle:
                %   obstacle, color index = 1, red
                %   free space, color index > 1, greyscale 
                
                % find maximum distance, ignore infinite values in
                % obstacles
                d = opt.distance(isfinite(opt.distance));
                d = d + 2;   % minimum distance is cmap=2 or black
                maxdist = max(d(:));

                % create the color map
                %  1 = red (obstacle)
                %  2 = black (zero distance)
                %  max = white (maximum distance)
                cmap = [1 0 0; opt.colormap(ceil(maxdist))];
                
                % distance of 0 has display value of 2
                opt.distance = opt.distance + 2;
                
                % invalid distances show as black
                opt.distance(isnan(opt.distance)) = 2;
                
                % ensure obstacles appear as red
                opt.distance(occgrid > 0) = 1;
                
                % display it with colorbar
                image(opt.distance, 'CDataMapping', 'direct');
                set(gcf, 'Renderer', 'Zbuffer')
                colormap(cmap)
                cb = colorbar;
                cb.Label.String = 'Distance to goal (cells)';
                brighten(opt.beta)
            end
            
            % label the grid
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            grid on
            hold on
        end
        
        function plot_fg(nav, varargin)
        %Navigation.plot_fg  Visualization foreground
        %
        % N.plot_fg(OPTIONS) displays the start and goal locations if specified.
        % By default the goal is a pentagram and start is a circle.
        %
        % N.plot_fg(P, OPTIONS) as above but overlays the points along the path (2xM) matrix.
        % 
        % Options::
        %  'pathmarker',M    Options to draw a path point
        %  'startmarker',M   Options to draw the start marker
        %  'goalmarker',M    Options to draw the goal marker
        %
        % Notes::
        % - In all cases M is a single string eg. 'r*' or a cell array of MATLAB LineSpec options.
        % - Typically used after a call to plot_bg().
        %
        % See also Navigation.plot_bg.
            
            opt.pathmarker =  {};
            opt.startmarker = {};
            opt.goalmarker =  {};
            opt.goal = true;
            
            pathmarker =  {'g.', 'MarkerSize', 12};
            startmarker = {'bo','MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', 'MarkerSize', 12};
            goalmarker =  {'bp', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w', 'MarkerSize', 18};
            
            [opt,args] = tb_optparse(opt, varargin);
                    
                        
            % overlay a path if provided
            if ~isempty(args) && isnumeric(args{1})
                p = args{1};
                if numcols(p) < 2
                    error('expecting Nx2 or Nx3 matrix of points');
                end
                if numcols(p) == 2
                    plot(p(:,1), p(:,2), pathmarker{:}, ...
                        opt.pathmarker{:}, 'Tag', 'path');
                else
                    plot3(p(:,1), p(:,2), p(:,3), pathmarker{:}, ...
                        opt.pathmarker{:}, 'Tag', 'path');
                end
            end
            
            % mark start and goal if requested
            if length(nav.goal) == 2
                if opt.goal && ~isempty(nav.goal)
                    plot(nav.goal(1), nav.goal(2), ...
                        goalmarker{:}, opt.goalmarker{:}, 'Tag', 'goal');
                end
                if opt.goal && ~isempty(nav.start)
                    plot(nav.start(1), nav.start(2), ...
                        startmarker{:}, opt.startmarker{:}, 'Tag', 'start');
                end
            else
                if opt.goal && ~isempty(nav.goal)
                    plot3(nav.goal(1), nav.goal(2), nav.goal(3)+0.1, ...
                        goalmarker{:}, opt.goalmarker{:}, 'Tag', 'goal');
                end
                if opt.goal && ~isempty(nav.start)
                    plot3(nav.start(1), nav.start(2), nav.start(3)+0.1, ...
                        startmarker{:}, opt.startmarker{:}, 'Tag', 'start');
                end
            end

            hold off
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
        
        
        function setgoal(nav, goal)
            
            if isempty(goal)
                nav.plot();
                disp('select goal location'); beep
                goal = round(ginput(1));
            end
            % make upright
            nav.goal = goal(:);
            
            % check if reachable
            if nav.isoccupied(nav.goal)
                error('Navigation:checkquery:badarg', 'goal location inside obtacle');
            end
        end
        
        function checkquery(nav, start, goal)
            
            % if any of start or goal are [], prompt the user to select
            if isempty(start)
                nav.plot();
                disp('Select start location'); beep
                start = round(ginput(1));
            end
            
            if nargin == 3
                % this planner supports a query with a goal
                if isempty(goal)
                    nav.plot();
                    disp('Select goal location'); beep
                    goal = round(ginput(1));
                end
            end
            
            % make start and goal column vectors
            nav.start = start(:);
            if nargin == 3
                % this planner supports a query with a goal
                nav.goal = goal(:);
            end
            
            % check if reachable
            assert(~nav.isoccupied(nav.start(1:2)), 'Navigation:checkquery:badarg', 'start location inside obstacle');
            
            if nargin == 3
                % make upright
                nav.goal = goal(:);
                
                % check if reachable
                assert(~nav.isoccupied(nav.goal(1:2)), 'Navigation:checkquery:badarg', 'goal location inside obstacle');
            end
        end
        
        
        function occ = isoccupied(nav, x, y)
            %Navigation.isoccupied Test if grid cell is occupied
            %
            % N.isoccupied(POS) is true if there is a valid grid map and the
            % coordinates given by the columns of POS (2xN) are occupied.
            %
            % N.isoccupied(X,Y) as above but the coordinates given separately.
            %
            % Notes:
            % -  X and Y are Cartesian rather than MATLAB row-column coordinates.
            
            if isempty(nav.occgridnav)
                occ = false;
                return
            end
            
            if nargin == 2
                % isoccupied(p)
                if numel(x) == 2
                    x = x(:);
                end
                assert(size(x,1) == 2, 'RTB:Navigation:isoccupied', 'P must have 2 rows');
                pos = x;
            else
                % isoccupied(x,y)
                assert(numel(x) == numel(y), 'RTB:Navigation:isoccupied', 'X and Y must be same length');
                pos = [x(:)'; y(:)'];
            end
            
            % convert from world coordinates to grid coordinates
            pos = round( nav.w2g * pos );
            
            % find all those that lie in the map
            k = pos(1,:) > 0 & pos(1,:) <= size(nav.occgrid,2) & pos(2,:) > 0 & pos(2,:) <= size(nav.occgrid,1);
            
            % get the indices into the map
            i = sub2ind(size(nav.occgrid), pos(2,k), pos(1,k));
            
            occ = ones(1, size(pos,2), 'logical'); % by default all occupied (true)
            occ(k) = nav.occgridnav(i) > 0;
        end
        
        function goal_change(nav)
            %Navigation.goal_change Notify change of goal
            %
            % Invoked when the goal property of the object is changed.  Typically this
            % is overriden in a subclass to take particular action such as invalidating
            % a costmap.
        end
        
        function navigate_init(nav, start)
            %Navigation.navigate_init Notify start of path
            %
            % N.navigate_init(START) is called when the query() method is invoked.
            % Typically overriden in a subclass to take particular action such as
            % computing some path parameters. START (2x1) is the initial position for this
            % path, and nav.goal (2x1) is the final position.
            %
            % See also Navigate.query.
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
        % - Provides an independent sequence of random numbers that does not
        %   interfere with any other randomised algorithms that might be used.
        %
        % See also Navigation.randi, Navigation.randn, rand, RandStream.
            r = nav.randstream.rand(varargin{:});
        end

        function r = randn(nav, varargin)
        %Navigation.randn Normally distributed random number
        %
        % R = N.randn() returns a normally distributed random number from
        % a private random number stream.
        %
        % R = N.randn(M) as above but returns a matrix (MxM) of random numbers.
        %
        % R = N.randn(L,M) as above but returns a matrix (LxM) of random numbers.
        %
        % Notes::
        % - Accepts the same arguments as randn().
        % - Seed is provided to Navigation constructor.
        % - Provides an independent sequence of random numbers that does not
        %   interfere with any other randomised algorithms that might be used.
        %
        % See also Navigation.rand, Navigation.randi, randn, RandStream.
            r = nav.randstream.randn(varargin{:});
        end

        function r = randi(nav, varargin)
        %Navigation.randi Integer random number
        %
        % I = N.randi(RM) returns a uniformly distributed random integer in the 
        % range 1 to RM from a private random number stream.
        %
        % I = N.randi(RM, M) as above but returns a matrix (MxM) of random integers.
        %
        % I = N.randn(RM, L,M) as above but returns a matrix (LxM) of random integers.
        %
        % Notes::
        % - Accepts the same arguments as randi().
        % - Seed is provided to Navigation constructor.
        % - Provides an independent sequence of random numbers that does not
        %   interfere with any other randomised algorithms that might be used.
        %
        % See also Navigation.rand, Navigation.randn, randi, RandStream.
            r = nav.randstream.randi(varargin{:});
        end
        
        function verbosity(nav, v)
        %Navigation.verbosity Set verbosity
        %
        % N.verbosity(V) set verbosity to V, where 0 is silent and greater
        % values display more information.
            nav.verbose = v;
        end
       
        
        function message(nav, varargin)
        %Navigation.message Print debug message
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
            fprintf('\b%c', spinchars( mod(nav.spincount, length(spinchars))+1 ) );
        end
        
    end
    
    methods (Static)
        
        function show_distance(d)
            d(isinf(d)) = NaN;
            clf
            ax = gca;
            colormap(gray(256));

            ax.CLimMode = 'Manual';
            ax.CLim = [0 max(d(:))];
            image(d, 'CDataMapping', 'scaled');
            ax.YDir = 'normal';
            grid on; xlabel('X'); ylabel('Y');
            drawnow
        end

        function h = progress_init(title)
            h = waitbar(0, title, ...
                'CreateCancelBtn', 'setappdata(gcbf, ''canceling'', 1)');
        end
        
        function progress(h, x)
            waitbar(x, h);
        end
        
        function progress_delete(h)
            delete(h);
        end

    end % method

end % classde