%RTTRDEMO Trajectory demo


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
%
% http://www.petercorke.com
figure(2)
echo on
% The path will move the robot from its zero angle pose to the upright (or 
% READY) pose.
%
% First create a time vector, completing the motion in 2 seconds with a
% sample interval of 56ms.
    t = [0:.056:2];
pause % hit any key to continue
%
% A polynomial trajectory between the 2 poses is computed using jtraj()
%
    q = jtraj(qz, qr, t);
pause % hit any key to continue

%
% For this particular trajectory most of the motion is done by joints 2 and 3,
% and this can be conveniently plotted using standard MATLAB operations
    subplot(2,1,1)
    plot(t,q(:,2))
    title('Theta')
    xlabel('Time (s)');
    ylabel('Joint 2 (rad)')
    subplot(2,1,2)
    plot(t,q(:,3))
    xlabel('Time (s)');
    ylabel('Joint 3 (rad)')


    pause % hit any key to continue
%
% We can also look at the velocity and acceleration profiles.  We could 
% differentiate the angle trajectory using diff(), but more accurate results 
% can be obtained by requesting that jtraj() return angular velocity and 
% acceleration as follows
    [q,qd,qdd] = jtraj(qz, qr, t);
%
%  which can then be plotted as before

    subplot(2,1,1)
    plot(t,qd(:,2))
    title('Velocity')
    xlabel('Time (s)');
    ylabel('Joint 2 vel (rad/s)')
    subplot(2,1,2)
    plot(t,qd(:,3))
    xlabel('Time (s)');
    ylabel('Joint 3 vel (rad/s)')
pause(2)
% and the joint acceleration profiles
    subplot(2,1,1)
    plot(t,qdd(:,2))
    title('Acceleration')
    xlabel('Time (s)');
    ylabel('Joint 2 accel (rad/s2)')
    subplot(2,1,2)
    plot(t,qdd(:,3))
    xlabel('Time (s)');
    ylabel('Joint 3 accel (rad/s2)')
pause % any key to continue
echo off
