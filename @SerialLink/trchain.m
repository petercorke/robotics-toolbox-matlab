%SERIALLINK.TRCHAIN Convert to elementary transform sequence
%
% S = R.TRCHAIN(OPTIONS) is a sequence of elementary transforms that describe the
% kinematics of the serial link robot arm.  The string S comprises a number
% of tokens of the form X(ARG) where X is one of Tx, Ty, Tz, Rx, Ry, or Rz.
% ARG is a joint variable, or a constant angle or length dimension.
%
% For example:
%        >> mdl_puma560
%        >> p560.trchain
%        ans =
%        Rz(q1)Rx(90)Rz(q2)Tx(0.431800)Rz(q3)Tz(0.150050)Tx(0.020300)Rx(-90)
%        Rz(q4)Tz(0.431800)Rx(90)Rz(q5)Rx(-90)Rz(q6)
%
% Options::
% '[no]deg'    Express angles in degrees rather than radians (default deg)
% 'sym'        Replace length parameters by symbolic values L1, L2 etc.
%
% See also trchain, trotx, troty, trotz, transl.

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

function s = trchain(robot, varargin)
    
    opt.sym = false;
    opt.deg = true;
    opt = tb_optparse(opt, varargin);
    
    if opt.deg
        conv = 180/pi;
    else
        conv = 1;
    end
    
    s = '';
    varcount = 1;
    
    for j=1:robot.n
        L = robot.links(j);
        
        if L.isrevolute()
            s = append(s, 'Rz(q%d)', j);
            if L.d ~= 0
                if opt.sym
                    s = append(s, 'Tz(L%d)', varcount);
                    varcount = varcount+1;
                else
                    s = append(s, 'Tz(%g)', L.d);
                end
            end
        else
            if L.theta ~= 0
                s = append(s, 'Rz(%g)', (L.alpha*conv));
            end
            s = append(s, 'Tz(q%d)', j);
        end
        
        if L.a ~= 0
            if opt.sym
                s = append(s, 'Tx(L%d)', varcount);
                varcount = varcount+1;
            else
                s = append(s, 'Tx(%g)', L.a);
            end
            
        end
        if L.alpha ~= 0
            s = append(s, 'Rx(%g)', (L.alpha*conv));
        end
    end
end
    
    
    function s = append(s, fmt, j)
        s = strcat(s, sprintf(fmt, j));
    end
