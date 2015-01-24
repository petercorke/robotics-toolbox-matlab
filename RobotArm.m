%RobotArm Serial-link robot arm class
%
% A subclass of SerialLink than includes an interface to a physical robot.
%
% Methods::
%
%  plot          display graphical representation of robot
%-
%  teach         drive the physical and graphical robots
%  mirror        use the robot as a slave to drive graphics
%-
%  jmove         joint space motion of the physical robot
%  cmove         Cartesian space motion of the physical robot
%
% plus all other methods of SerialLink
%
% Properties::
%
% as per SerialLink class
%
% Note::
%  - the interface to a physical robot, the machine, should be an abstract
%  superclass but right now it isn't
%  - RobotArm is a subclass of SerialLink.
%  - RobotArm is a reference (handle subclass) object.
%  - RobotArm objects can be used in vectors and arrays
%
% Reference::
% - http://www.petercorke.com/doc/robotarm.pdf
% - Robotics, Vision & Control, Chaps 7-9,
%   P. Corke, Springer 2011.
% - Robot, Modeling & Control,
%   M.Spong, S. Hutchinson & M. Vidyasagar, Wiley 2006.
%
% See also Machine, SerialLink, Link, DHFactor.

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

classdef RobotArm < SerialLink

    properties
        machine
        ngripper
    end

    methods
        function ra = RobotArm(robot, machine, varargin)
            %RobotArm.RobotArm Construct a RobotArm object
            %
            % RA = RobotArm(L, M, OPTIONS) is a robot object defined by a vector of
            % Link objects L with a physical robot interface M represented by an object
            % of class Machine.
            %
            % Options::
            %
            %  'name', name            set robot name property
            %  'comment', comment      set robot comment property
            %  'manufacturer', manuf   set robot manufacturer property
            %  'base', base            set base transformation matrix property
            %  'tool', tool            set tool transformation matrix property
            %  'gravity', g            set gravity vector property
            %  'plotopt', po           set plotting options property
            %
            % See also SerialLink.SerialLink, Arbotix.Arbotix.

            ra = ra@SerialLink(robot, varargin{:});
            ra.machine = machine;
            
            ra.ngripper = machine.nservos - ra.n;
        end

        function delete(ra)
            %RobotArm.delete Destroy the RobotArm object
            %
            % RA.delete() closes and destroys the machine interface object and the RobotArm
            % object.
            
            % attempt to destroy the machine interfaace
            try
                ra.machine.disconnect();
                delete(ra.machine);
            catch
            end
            
            % cleanup the parent object
            delete@SerialLink(ra);
        end
        
        function jmove(ra, qf, t)
            %RobotArm.jmove Joint space move
            %
            % RA.jmove(QD) moves the robot arm to the configuration specified by
            % the joint angle vector QD (1xN).
            %
            % RA.jmove(QD, T) as above but the total move takes T seconds.
            %
            % Notes::
            % - A joint-space trajectory is computed from the current configuration to QD.
            %
            % See also RobotArm.cmove, Arbotix.setpath.
            
            if nargin < 3
                t = 3;
            end
            
            q0 = ra.getq();
            qt = jtraj(q0, qf, 20);
            
            ra.machine.setpath(qt, t/20);
        end
        
        function cmove(ra, T, varargin)
            %RobotArm.cmove Cartesian space move
            %
            % RA.cmove(T) moves the robot arm to the pose specified by
            % the homogeneous transformation (4x4).
            %
            % Notes::
            % - A joint-space trajectory is computed from the current configuration to
            %   QD using the jmove() method.
            % - If the robot is 6-axis with a spherical wrist inverse kinematics are
            %   computed using ikine6s() otherwise numerically using ikine().
            %
            % See also RobotArm.jmove, Arbotix.setpath.
            if ra.isspherical()
                q = ra.ikine6s(T, varargin{:});
            else
                q = ra.ikine(T, ra.getq(), [1 1 1  1 0 0]);
            end
            ra.jmove(q);
        end
        
        function q = getq(ra)
            %RobotArm.getq Get the robot joint angles
            %
            % Q = RA.getq() is a vector (1xN) of robot joint angles.
            %
            % Notes::
            % - If the robot has a gripper, its value is not included in this vector.
            
            q = ra.machine.getpos();
            q = q(1:ra.n);
        end
        
        function mirror(ra)
            %RobotArm.mirror Mirror the robot pose to graphics
            %
            % RA.mirror() places the robot arm in relaxed mode, and as it is moved by
            % hand the graphical animation follows.
            %
            % See also SerialLink.teach, SerialLink.plot.
            
            h = msgbox('The robot arm will go to relaxed mode, type q in the figure window to exit', ...
                'Mirror mode', 'warn');
            
            ra.machine.relax();
            while true
                if get(gcf,'CurrentCharacter') == 'q'
                    break
                end;
                
                q = ra.machine.getpos();
                ra.plot(q(1:ra.n));
                
            end
            ra.machine.relax([], false);
            
            delete(h);
        end
        
        function teach(ra)
            %RobotArm.teach Teach the robot
            %
            % RA.teach() invokes a simple GUI to allow joint space motion, as well
            % as showing an animation of the robot on screen.
            %
            % See also SerialLink.teach, SerialLink.plot.
            
            q0 = ra.machine.getpos();
            
            teach@SerialLink(ra, 'q0', q0(1:ra.n), ...
                'callback', @(q) ra.machine.setpos([q q0(ra.n+1)]) );
        end
        
        function gripper(ra, open)
            %RobotArm.gripper Control the robot gripper
            %
            % RA.gripper(C) sets the robot gripper according to C which is 0 for closed
            % and 1 for open.
            %
            % Notes::
            % - Not all robots have a gripper.
            % - The gripper is assumed to be the last servo motor in the chain.
            if open < 0 || open > 1
                error('RTB:RobotArm:badarg', 'gripper control must be in range 0 to 1');
            end
            
            if ra.ngripper == 0
                error('RTB:RobotArm:nofunc', 'robot has no gripper');
            end
            
            griplimits = ra.machine.gripper;
            a = open*griplimits(1) + (1-open)*griplimits(2)
            ra.machine.setpos(ra.n+1, a);
        end
    end
end
