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

% Generate a Reeds-Shepp path for 3-point turn
dl = 0.05;
q0 = [0 0 0]'; qf = [0 0 pi]';
maxcurv = 1/5;   % 5m turning circle
rs = ReedsShepp(q0, qf, maxcurv, dl)

% set up a vehicle model
[car.image,~,car.alpha] = imread('car2.png');
car.rotation = 180;
car.centre = [81,110];
car.centre = [648; 173];
car.length = 4.2;

% now animate
clf; plotvol([-4 8 -6 6])

a = gca;
a.XLimMode = 'manual';
a.YLimMode = 'manual';
set(gcf, 'Color', 'w')
grid on
a = gca;
xyzlabel

plot_vehicle(rs.path, 'model', car, 'trail', 'r:');





