%DISTANCEXFORM Distance transform of occupancy grid
%
% D = DISTANCEXFORM(WORLD, GOAL) is the distance transform of the occupancy 
% grid WORLD with respect to the specified goal point GOAL = [X,Y].  The
% cells of the grid have values of 0 for free space and 1 for obstacle.
%
% D = DISTANCEXFORM(WORLD, GOAL, METRIC) as above but specifies the distance
% metric as  either 'cityblock' or 'Euclidean' (default).
%
% D = DISTANCEXFORM(WORLD, GOAL, METRIC, SHOW) as above but shows an animation
% of the distance transform being formed, with a delay of SHOW seconds between
% frames.
%
% Notes::
% - The Machine Vision Toolbox function imorph is required.
% - The goal is [X,Y] not MATLAB [row,col].
%
% See also IMORPH, DXform.


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

function d = distancexform(world, goal, metric, show)
    
    if exist('imorph', 'file') ~= 3
        error('Machine Vision Toolbox is required by this function');
    end

    if nargin < 4
        show = 0;
    end

    % set up the distance metrics
    if nargin < 3
        metric = 'cityblock';
    end

    if strncmpi(metric, 'cityblock', length(metric))
        m = ones(3,3);
        m(2,2) = 0;
    elseif strncmpi(metric, 'euclidean', length(metric))
        r2 = sqrt(2);
        m = [r2 1 r2; 1 0 1; r2 1 r2];
    else
        error('unknown distance metric');
    end

    % for the imorph primitive we need to set the target pixel to 0,
    % obstacles to NaN and the rest to Inf.

    if ~isempty(goal)
        % specific goal point
        if world(goal(2), goal(1)) > 0
            error('goal inside obstacle')
        end
        % point goal case
        world(world>0) = NaN;
        world(world==0) = Inf;
        world(goal(2), goal(1)) = 0;
    else
        world = double(world);
        world(world==0) = Inf;
        world(isfinite(world)) = 0;
        idisp(world)
    end


    count = 0;
    while 1
        world = imorph(world, m, 'plusmin');
        count = count+1;
        if show
            cmap = [1 0 0; gray(256)];
            colormap(cmap)
            image(world+1, 'CDataMapping', 'direct');
            set(gca, 'Ydir', 'normal');
            xlabel('x');
            ylabel('y');
            pause(show);
        end

        if ~any(isinf(world(:)))
            % stop if no Inf's left in the map
            break;
        end
    end

    if show
        fprintf('%d iterations\n', count);
    end

    d = world;
