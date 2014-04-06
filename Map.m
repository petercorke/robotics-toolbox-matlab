%map Map of planar point features
%
% A Map object represents a square 2D environment with a number of landmark
% feature points.
%
% Methods::
%   plot      Plot the feature map
%   feature   Return a specified map feature
%   display   Display map parameters in human readable form
%   char      Convert map parameters to human readable string
%
% Properties::
%   map         Matrix of map feature coordinates 2xN
%   dim         The dimensions of the map region x,y in [-dim,dim]
%   nfeatures   The number of map features N
%
% Examples::
%
% To create a map for an area where X and Y are in the range -10 to +10 metres
% and with 50 random feature points
%        map = Map(50, 10);
% which can be displayed by
%        map.plot();
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
% See also RangeBearingSensor, EKF.

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

classdef Map < handle
% TODO:
% add a name property, show in char()

    properties
        map    % map features
        dim         % map dimension
        nfeatures   % number of features in map

        verbose
    end

    methods

        % constructor
        function map = Map(nfeatures, varargin)
        %Map.Map Map of point feature landmarks
        %
        % M = Map(N, DIM, OPTIONS) is a Map object that represents N random point features
        % in a planar region bounded by +/-DIM in the x- and y-directions.
        %
        % Options::
        % 'verbose'    Be verbose
            
            opt = [];
            [opt,args] = tb_optparse(opt, varargin);
            map.verbose = opt.verbose;

            if ~isempty(args) && isnumeric(args{1})
                dim = args{1};
            else
                dim = 10;
            end
            map.dim = dim;
            map.nfeatures = nfeatures;
            map.map = dim * (2*rand(2, nfeatures)-1);
            map.verbose = false;
        end

        function f = feature(map, k)
            %Map.feature Return the specified map feature
            %
            % F = M.feature(K) is the coordinate (2x1) of the K'th feature.
            f = map.map(:,k);
        end

        function plot(map, varargin)
            %Map.plot Plot the map
            %
            % M.plot() plots the feature map in the current figure, as a square
            % region with dimensions given by the M.dim property.  Each feature
            % is marked by a black diamond.
            %
            % M.plot(LS) plots the feature map as above, but the arguments LS
            % are passed to plot and override the default marker style.
            %
            % Notes::
            % - The plot is left with HOLD ON.
            clf
            d = map.dim;
            axis([-d d -d d]);
            xlabel('x');
            ylabel('y');

            if nargin == 1
                args = {'kd'};
            else
                args = varargin;
            end
            plot(map.map(1,:)', map.map(2,:)', args{:});
            grid on
            hold on
        end

        function show(map, varargin)
        %map.SHOW Show the feature map
        %
        % Notes::
        % - Deprecated, use plot method.
            warning('show method is deprecated, use plot() instead');
            map.plot(varargin{:});
        end

        function verbosity(map, v)
            %map.verbosity Set verbosity
            %
            % M.verbosity(V) set verbosity to V, where 0 is silent and greater
            % values display more information.
            map.verbose = v;
        end
            
        function display(map)
            %map.display Display map parameters
            %
            % M.display() display map parameters in a compact
            % human readable form.
            %
            % Notes::
            % - this method is invoked implicitly at the command line when the result
            %   of an expression is a Map object and the command has no trailing
            %   semicolon.
            %
            % See also map.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(map) );
        end % display()

        function s = char(map)
        %map.char Convert vehicle parameters and state to a string
        %
        % s = M.char() is a string showing map parameters in 
        % a compact human readable format. 
            s = 'Map object';
            s = char(s, sprintf('  %d features', map.nfeatures));
            s = char(s, sprintf('  dimension %.1f', map.dim));
        end

    end % method

end % classdef
