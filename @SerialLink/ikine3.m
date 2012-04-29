%SerialLink.ikine3 Inverse kinematics for 3-axis robot with no wrist
%
% Q = R.ikine3(T) is the joint coordinates corresponding to the robot
% end-effector pose T represented by the homogenenous transform.  This
% is a analytic solution for a 3-axis robot (such as the first three joints
% of a robot like the Puma 560).
%
% Q = R.ikine3(T, CONFIG) as above but specifies the configuration of the arm in
% the form of a string containing one or more of the configuration codes:
%
% 'l'   arm to the left (default)
% 'r'   arm to the right
% 'u'   elbow up (default)
% 'd'   elbow down
%
% Notes::
% - The same as IKINE6S without the wrist.
% - The inverse kinematic solution is generally not unique, and 
%   depends on the configuration string.
% - Joint offsets, if defined, are added to the inverse kinematics to 
%   generate Q.
%
% Reference::
%
% Inverse kinematics for a PUMA 560 based on the equations by Paul and Zhang
% From The International Journal of Robotics Research
% Vol. 5, No. 2, Summer 1986, p. 32-44
%
%
% Author::
% Robert Biro with Gary Von McMurray,
% GTRI/ATRP/IIMB,
% Georgia Institute of Technology
% 2/13/95
%
% See also SerialLink.FKINE, SerialLink.IKINE.

function theta = ikine3(robot, T, varargin)

    if ~strncmp(robot.config, 'RRR', 3)
        error('Solution only applicable for 6DOF all-revolute manipulator');
    end

    if robot.mdh ~= 0
        error('Solution only applicable for standard DH conventions');
    end

    if ndims(T) == 3
        theta = zeros(size(T,3),robot.n);
        for k=1:size(T,3)
            theta(k,:) = ikine3(robot, T(:,:,k), varargin{:});
        end
        return;
    end
    L = robot.links;
    a2 = L(2).a;
    a3 = L(3).a;

    d3 = L(3).d;

    if ~ishomog(T)
        error('T is not a homog xform');
    end

    % undo base transformation
    T = robot.base \ T;

    % The following parameters are extracted from the Homogeneous 
    % Transformation as defined in equation 1, p. 34

    Px = T(1,4);
    Py = T(2,4);
    Pz = T(3,4);

    % The configuration parameter determines what n1,n2 values are used
    % and how many solutions are determined which have values of -1 or +1.

    if nargin < 3
        configuration = '';
    else
        configuration = lower(varargin{1});
    end

    % default configuration

    n1 = -1;    % L
    n2 = -1;    % U
    if ~isempty(strfind(configuration, 'l'))
        n1 = -1;
    end
    if ~isempty(strfind(configuration, 'r'))
        n1 = 1;
    end
    if ~isempty(strfind(configuration, 'u'))
        if n1 == 1
            n2 = 1;
        else
            n2 = -1;
        end
    end
    if ~isempty(strfind(configuration, 'd'))
        if n1 == 1
            n2 = -1;
        else
            n2 = 1;
        end
    end

    %
    % Solve for theta(1)
    % 
    % r is defined in equation 38, p. 39.
    % theta(1) uses equations 40 and 41, p.39, 
    % based on the configuration parameter n1
    %

    r=sqrt(Px^2 + Py^2);
    if (n1 == 1)
        theta(1)= atan2(Py,Px) + asin(d3/r);
    else
        theta(1)= atan2(Py,Px) + pi - asin(d3/r);
    end

    %
    % Solve for theta(2)
    %
    % V114 is defined in equation 43, p.39.
    % r is defined in equation 47, p.39.
    % Psi is defined in equation 49, p.40.
    % theta(2) uses equations 50 and 51, p.40, based on the configuration 
    % parameter n2
    %

    V114= Px*cos(theta(1)) + Py*sin(theta(1));
    r=sqrt(V114^2 + Pz^2);
    Psi = acos((a2^2-d4^2-a3^2+V114^2+Pz^2)/(2.0*a2*r));
    if ~isreal(Psi)
        warning('RTB:ikine3:notreachable', 'point not reachable');
        theta = [NaN NaN NaN NaN NaN NaN];
        return
    end
    theta(2) = atan2(Pz,V114) + n2*Psi;

    %
    % Solve for theta(3)
    %
    % theta(3) uses equation 57, p. 40.
    %

    num = cos(theta(2))*V114+sin(theta(2))*Pz-a2;
    den = cos(theta(2))*Pz - sin(theta(2))*V114;
    theta(3) = atan2(a3,d4) - atan2(num, den);

    % remove the link offset angles
    for i=1:robot.n   %#ok<*AGROW>
        theta(i) = theta(i) - L(i).offset;
    end
