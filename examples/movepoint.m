
% Copyright (C) 1993-2014, by Peter I. Corke
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
xg = [5 5]; % goal position
clf
axis([0 10 0 10]);
hold on
xyzlabel
grid on
xc = 5; yc = 5;
N = 8;
radius = 3;

for i=1:N
    th = (i-1)*2*pi/N;
    x0 = [xc+radius*cos(th) yc+radius*sin(th) th+pi/2];

    plot_vehicle(x0, 'r');
    r = sim('sl_drivepoint');
    y = r.find('yout');
    plot(y(:,1), y(:,2));
end
plot(xg(1), xg(2), '*')
