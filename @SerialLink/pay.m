%SerialLink.PAY Joint forces due to payload
%
% TAU = R.PAY(W, J) returns the generalised joint force/torques due to a
% payload wrench W (1x6) and where the manipulator Jacobian is J (6xN). 
%
% TAU = R.PAY(Q, W, F) as above but the Jacobian is calculated at pose Q
% (1xN) in the frame given by F which is '0' for world frame, 'n' for
% end-effector frame.
%
% Uses the formula TAU = J'W, where W is a wrench vector applied at the end
% effector, W = [Fx Fy Fz Mxx Myy Mzz]'.
%
% Trajectory operation::
%
% In the case Q is MxN then TAU is MxN where each row is the generalised
% force/torque at the pose given by corresponding row of Q.
%
% Notes::
% - Tool transforms are not taken into consideration.
%
% Author::
% Bryan Moutrie
%
% See also SerialLink.paycap, SerialLink.jacob0, SerialLink.jacobn.

% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.

function tauP = pay(varargin)
    
    if length(varargin) == 2
        w = varargin{1};
        J = varargin{2};
        n = size(J,2);
    elseif length(varargin) == 4
        robot = varargin{1};
        q = varargin{2};
        w = varargin{3};
        f = varargin{4};
        n = robot.n;
        J = zeros(6,n,size(q,1));
        if f == '0'
            for i= 1: size(q,1)
                J(:,:,i) = robot.jacob0(q(i,:));
            end
        elseif f == 'n'
            for i= 1: size(q,1)
                J(:,:,i) = robot.jacobn(q(i,:));
            end
        end
    end
    
    tauP = -reshape(J(:,:)'*w,n,[])';
    
end

