%JOY2TR Update transform from joystick
%
% T = JOY2TR(T, OPTIONS) updates the SE(3) homogeneous transform (4x4)
% according to spatial velocities sourced from a connected joystick device.
%
% Options::
% 'delay',D     Pause for D seconds after reading (default 0.1)
% 'scale',S     A 2-vector which scales joystick translational and 
%               rotational to rates (default [0.5m/s, 0.25rad/s])
% 'world'       Joystick motion is in the world frame
% 'tool'        Joystick motion is in the tool frame (default)
% 'rotate',R    Index of the button used to enable rotation (default 7)
%
% Notes::
% - Joystick axes 0,1,3 map to X,Y,Z or R,P,Y motion.
% - A joystick button enables the mapping to translation OR rotation.
% - A 'delay' of zero means no pause
% - If 'delay' is non-zero 'scale' maps full scale to m/s or rad/s.
% - If 'delay' is zero 'scale' maps full scale to m/sample or rad/sample.
%
% See also joystick.

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

function Tout = joy2tr(T, varargin)
    
    % parse the options
    opt.delay = 0.1;
    opt.rotate = 7;
    opt.scale = [0.5 0.25];
    opt.frame = {'tool', 'world'};
    opt.min = 0.006;
    
    opt = tb_optparse(opt, varargin);
    
    % get the raw joystick output
    [j,b] = joystick();
    
    % update a 6-vector of translational and rotational velocities
    vel = zeros(1,6);
    if b(7) == 0
        % translation mode
        vel(1:2) = j(1:2);
        vel(3) = j(4);
    else
        % rotation mode
        vel(4:5) = j(1:2);
        vel(6) = j(4);
    end
    
    % values below threshold are set to zero
    vel(abs(vel) < opt.min) = 0;
    
    % apply scaling factors
    scale = opt.scale;
    
    if opt.delay > 0
        % normalize scaling factors by time
        pause(opt.delay);
        scale = scale * opt.delay;
    end
    
    vel(1:3) = vel(1:3) * scale(1);
    vel(4:6) = vel(4:6) * scale(2);
    
    % compute the incremental motion
    dT = transl(vel(1:3)) * rpy2tr(vel(4:6));

    
    % and apply it to the input transform
    switch opt.frame
        case 'tool'
            Tout = T * dT;
        case 'world'
            Tout = dT * T;
    end
    
    % normalize just to be safe
    Tout = trnorm(Tout);
