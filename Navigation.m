%Navigation Navigation superclass
%
% An abstract superclass for implementing navigation classes.  
%
% nav = Navigation(occgrid, options) is an instance of the Navigation object.
%
% Methods::
%   visualize   display the occupancy grid
%   plan        plan a path to goal
%   path        return/animate a path from start to goal
%   display     print the parameters in human readable form
%   char        convert the parameters to a human readable string
%
% Properties (read only)::
%   occgrid   occupancy grid representing the navigation environment
%   goal      goal coordinate
%
% Methods to be provided in subclass::
%   goal_set        set the goal
%   world_set       set the occupancy grid
%   navigate_init
%   plan            generate a plan for motion to goal
%   next            returns coordinate of next point on path
% 
% Notes::
% - subclasses the Matlab handle class which means that pass by reference semantics
%   apply.
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

classdef Navigation < handle

    properties
        occgrid     % occupancy grid
        goal        % goal coordinate

        navhook     % function handle, called on each navigation iteration
        verbose     % verbosity
        seed            % current random seed
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
        % N = Navigation(OCCGRID, options) is a Navigation object that holds an
        % occupancy grid OCCGRID.  A number of options can be be passed.
        %
        % Options::
        %  'navhook',F   Specify a function to be called at every step of path
        %  'seed', s     Specify an initial random number seed
        %  'goal', g     Specify the goal point
        %  'verbose'     Display debugging information
            
            if nargin >= 1 && isnumeric(varargin{1})
                nav.occgrid = varargin{1};
                varargin = varargin(2:end);
            end
            
            % default values of options
            opt.verbose = false;
            opt.navhook = [];
            opt.seed = [];
            opt.goal = [];
            
            [opt,args] = tb_optparse(nav, varargin);
            
            % save current random seed so we can repeat the expt
            defaultStream = RandStream.getDefaultStream;
            if isempty(opt.seed)
                nav.seed = defaultStream.State;
            else
                defaultStream.State = opt.seed;
            end
            nav.verbose = opt.verbose;
            nav.navhook = opt.navhook;
            if ~isempty(opt.goal)
                nav.goal = opt.goal
            end
        end

        % set the occupancy grid
        %  can be overriden in a subclass
        function occgrid_set(nav, og)
            nav.occgrid = og;
        end

        function set.goal(nav, goal)
            disp('in set.goal');
            if ~isempty(nav.occgrid) && nav.occgrid( goal(2), goal(1)) == 1
                error('Navigation: cant set goal inside obstacle');
            else
                nav.goal = goal(:);
            end
            
            nav.goal_set(goal);
        end
        
        % invoked when goal changes
        %  can be overriden in a subclass
        function goal_set(nav, goal)
            disp('in base class goal_set');
        end
        


        function pp = path(nav, start)
            %Navigation.path Follow path from start to goal
            %
            % N.path(START) animates the robot moving from START to the goal (which is a property of 
            % the object).
            %
            % N.path() display the occupancy grid, prompt the user to click a start location,
            % then compute a path from this point to the goal (which is a property of 
            % the object).
            %
            % X = N.path(START) returns the path from START to the goal (which is a property of 
            % the object).
            %
            % The method performs the following steps:
            %
            %  - get start position interactively if not given
            %  - initialized navigation, invoke method N.navigate_init()
            %  - visualize the environment, invoke method N.visualize()
            %  - iterate on the next() method of the subclass
            %
            % See also Navigation.visualize, Navigation.goal.

            % if no start point given, display the map, and prompt the user to select
            % a start point
            if nargin < 2
                % display the world
                nav.visualize();

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
                nav.visualize();
                hold on
            end
            
            nav.navigate_init(start);

            p = [];
            % robot is a column vector
            robot = start;

            % iterate using the next() method until we reach the goal
            while true
                if nargout == 0
                    plot(robot(1), robot(2), 'g.');
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
        %Navigation.visualize  Visualize navigation environment
        %
        % N.visualize() displays the occupancy grid in a new figure.
        %
        % N.visualize(P) displays the occupancy grid in a new figure, and
        % shows the path points P which is an Nx2 matrix.
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
                colormap(cmap)
                colorbar
            end
            
            % label the grid
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            grid on
            hold on
            
            if length(args) > 0
                p = args{1};
                if numcols(p) ~= 2
                    error('expecting Nx2 matrix of points');
                end
                plot(p(:,1), p(:,2), 'g.');
            end
            
            if ~isempty(nav.goal) && opt.goal
                plot(nav.goal(1), nav.goal(2), 'bd', 'MarkerFaceColor', 'b');
            end
            hold off
        end

        % initialize navigation for this starting point
        function navigate_init(nav, start)
        end


        function display(nav)
        %Navigation.display Display status of navigation object
        %
        % N.display() display the state of the navigation object in 
        % human-readable form.
        %
        % Notes::
        % - this method is invoked implicitly at the command line when the result
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
        %Navigation.char Convert navigation object to string
        %
        % N.char() is a string representing the state of the navigation 
        % object in human-readable form.
            s = [class(nav) ' navigation class:'];
            s = strvcat(s, sprintf('  occupancy grid: %dx%d', size(nav.occgrid)));
            if ~isempty(nav.goal)
                s = strvcat(s, sprintf('   goal=%d,%d\n', nav.goal) );
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
            nav.navhook = navhook
        end
        
        function message(nav, str, varargin)
            if nav.verbose
                fprintf(['Navigation:: ' sprintf(str, varargin{:}) '\n']);
            end
        end
    end % method

end % classdef
