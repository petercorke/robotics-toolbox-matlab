%EX_TUMBLE Generate synthetic sensor data for a tumbling body
%
% Sets a number of workspace variables:
% t         time vector (1xN)
% w         angular velocity (3xN)
% truth     actual orientation as a unit quaternion (1xN)
% wm        simulated gyro output with bias (3xN)
% am        simulated accelerometer output with bias (3xN)
% mm        simulated magnetometer output with bias (3xN)
%
% Used on pages 81 and 89 of Robotics, Vision & Control, 2nd ed.

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


%% parameters (user adjustable)

dt = 0.05;          % sample interval

% make an asymmetric mass
J = diag([2 4 3]);
J(1,2) = -1;
J(2,1) = -1;
J(1,3) = -2;
J(3,1) = -2;
%eig(J)

% accelerometer
g0 = unit( [0, 0, 9.8]' );
gbias = 0.02*[2 -2 2]';  % bias 2% of norm

% magnetometer, use N E U data in nT  for Brisbane
m0 = unit( [28067.5, -5439.4, 44800.5]'*1e-9 );
mbias = 0.02*[-1 -1 2]';   % bias 2% of norm

% gyro
w0 = 0.2*[1 2 2]';
wbias = 0.1*[-1 2 -1]'; % bias 10% of max

%% simulation

% Solve Euler's rotational dynamic equation to get omega
[t,w] = ode45( @(t,w) -inv(J)*(cross(w, J*w)), [0:dt:20], w0);
w = w';  % one column per timestep

% Compute simulated sensor readings and true attitude
gm = zeros(3, numcols(w));
mm = zeros(3, numcols(w));

truth(1) = UnitQuaternion();

for k=1:numcols(w)-1
        iq = inv(truth(k));
        gm(:,k) = iq * g0 + gbias;  % sensor reading in body frame
        mm(:,k) = iq * m0 + mbias;  % sensor reading
        truth(k+1) = truth(k) .* UnitQuaternion.omega( w(:,k)*dt );
end

% add bias to measured angular velocity
wm = bsxfun(@plus, w, wbias);