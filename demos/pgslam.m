

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

%%begin

% We start by loading some data acquired on a mobile robot moving around
% MIT's Killian Court.

pg = PoseGraph('killian-small.toro');

% now we plot the data and we see that there are some error due to
% odometry, the long corridor appears as a triangle
clf; pg.plot()

% we optimize the pose graph
pg2 = pg.optimize();

% and then plot the optimized graph in a new figure
figure; pg2.plot

% and we see that the corridor now has a more sensible shape
