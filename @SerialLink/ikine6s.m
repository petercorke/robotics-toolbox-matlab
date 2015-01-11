%SerialLink.ikine6s Analytical inverse kinematics
%
% Q = R.ikine6s(T) is the joint coordinates (1xN) corresponding to the robot
% end-effector pose T represented by an SE(3) homogenenous transform (4x4).  This
% is a analytic solution for a 6-axis robot with a spherical wrist (the most
% common form for industrial robot arms).
%
% If T represents a trajectory (4x4xM) then the inverse kinematics is
% computed for all M poses resulting in Q (MxN) with each row representing
% the joint angles at the corresponding pose.
%
% Q = R.IKINE6S(T, CONFIG) as above but specifies the configuration of the arm in
% the form of a string containing one or more of the configuration codes:
%
% 'l'   arm to the left (default)
% 'r'   arm to the right
% 'u'   elbow up (default)
% 'd'   elbow down
% 'n'   wrist not flipped (default)
% 'f'   wrist flipped (rotated by 180 deg)
%
% Notes::
% - Treats a number of specific cases:
%  - Robot with no shoulder offset
%  - Robot with a shoulder offset (has lefty/righty configuration)
%  - Robot with a shoulder offset and a prismatic third joint (like Stanford arm)
%  - The Puma 560 arms with shoulder and elbow offsets (4 lengths parameters)
%  - The Kuka KR5 with many offsets (7 length parameters)
% - The inverse kinematic solution is generally not unique, and
%   depends on the configuration string.
% - Joint offsets, if defined, are added to the inverse kinematics to
%   generate Q.
% - Only applicable for standard Denavit-Hartenberg parameters
%
% Reference::
% - Inverse kinematics for a PUMA 560,
%   Paul and Zhang,
%   The International Journal of Robotics Research,
%   Vol. 5, No. 2, Summer 1986, p. 32-44
%
% Author::
% - The Puma560 case: Robert Biro with Gary Von McMurray,
%   GTRI/ATRP/IIMB, Georgia Institute of Technology, 2/13/95
% - Kuka KR5 case: Gautam Sinha,
%   Autobirdz Systems Pvt. Ltd.,  SIDBI Office,
%   Indian Institute of Technology Kanpur, Kanpur, Uttar Pradesh.
%
% See also SerialLink.FKINE, SerialLink.IKINE.

function theta = ikine6s(robot, T, varargin)
    
    if robot.mdh ~= 0
        error('RTB:ikine:notsupported','Solution only applicable for standard DH conventions');
    end
    
    if robot.n ~= 6
        error('RTB:ikine:notsupported','Solution only applicable for 6-axis robot');
    end
    
    % recurse over all poses in a trajectory
    if ndims(T) == 3
        theta = zeros(size(T,3),robot.n);
        for k=1:size(T,3)
            theta(k,:) = ikine6s(robot, T(:,:,k), varargin{:});
        end
        return;
    end

    
    if ~ishomog(T)
        error('RTB:ikine:badarg', 'T is not a homog xform');
    end
    
    L = robot.links;
    
    if ~robot.isspherical()
        error('RTB:ikine:notsupported', 'wrist is not spherical');
    end
    

        
    % The configuration parameter determines what n1,n2,n4 values are used
    % and how many solutions are determined which have values of -1 or +1.
    
    if nargin < 3
        configuration = '';
    else
        configuration = lower(varargin{1});
    end
    
    % default configuration
    
    sol = [1 1 1];  % left, up, noflip
    
    for c=configuration
        switch c
            case 'l'
                sol(1) = 1;
            case 'r'
                sol(1) = 2;
            case 'u'
                sol(2) = 1;
            case 'd'
                sol(2) = 2;
            case 'n'
                sol(3) = 1;
            case 'f'
                sol(3) = 2;
        end
    end
    

    % determine the arm structure and the relevant solution to use
    if isempty(robot.ikineType)
        if is_simple(L)
            robot.ikineType = 'nooffset';
        elseif is_puma(L)
            robot.ikineType = 'puma';
        elseif is_offset(L)
            robot.ikineType = 'offset';
        elseif is_rrp(L)
            robot.ikineType = 'rrp';
        else
            error('RTB:ikine6s:badarg', 'This kinematic structure not supported');
        end
    end
    
    % undo base and tool transformations
    T = inv(robot.base) * T * inv(robot.tool);
            
        
    %% now solve for the first 3 joints, based on position of the spherical wrist centre

    
    switch robot.ikineType
        case 'puma'
            % Puma model with shoulder and elbow offsets
            %
            % - Inverse kinematics for a PUMA 560,
            %   Paul and Zhang,
            %   The International Journal of Robotics Research,
            %   Vol. 5, No. 2, Summer 1986, p. 32-44
            %
            % Author::
            % Robert Biro with Gary Von McMurray,
            % GTRI/ATRP/IIMB,
            % Georgia Institute of Technology
            % 2/13/95
            
            a2 = L(2).a;
            a3 = L(3).a;
            d1 = L(1).d;
            d3 = L(3).d;
            d4 = L(4).d;
            

            
            % The following parameters are extracted from the Homogeneous
            % Transformation as defined in equation 1, p. 34
            
            Ox = T(1,2);
            Oy = T(2,2);
            Oz = T(3,2);
            
            Ax = T(1,3);
            Ay = T(2,3);
            Az = T(3,3);
            
            Px = T(1,4);
            Py = T(2,4);
            Pz = T(3,4) - d1;
            
            %
            % Solve for theta(1)
            %
            % r is defined in equation 38, p. 39.
            % theta(1) uses equations 40 and 41, p.39,
            % based on the configuration parameter n1
            %
            
            r=sqrt(Px^2 + Py^2);
            if sol(1) == 1
                theta(1)= atan2(Py,Px) + pi - asin(d3/r);
            else
                theta(1)= atan2(Py,Px) + asin(d3/r);
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
            if sol(2) == 1
                n2 = -1;
            else
                n2 = 1;
            end
            if sol(1) == 2
                n2 = -n2;
            end
            
            V114= Px*cos(theta(1)) + Py*sin(theta(1));
            r=sqrt(V114^2 + Pz^2);
            Psi = acos((a2^2-d4^2-a3^2+V114^2+Pz^2)/(2.0*a2*r));
            if ~isreal(Psi)
                warning('RTB:ikine6s:notreachable', 'point not reachable');
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
            
        case 'nooffset'
            a2 = L(2).a;
            a3 = L(3).a;
            d1 = L(1).d;
            
            px = T(1,4); py = T(2,4); pz = T(3,4);
            
            %%% autogenerated code
            if L(1).alpha < 0
                if sol(1) == 1
                    q(1) = angle(-px-py*1i);
                else
                    q(1) = angle(px+py*1i);
                end
                S1 = sin(q(1));
                C1 = cos(q(1));
                
                if sol(2) == 1
                    q(2) = -angle(a2*d1*-2.0+a2*pz*2.0-C1*a2*px*2.0i-S1*a2*py*2.0i)+angle(d1*pz*2.0i-a2^2*1i+a3^2*1i-d1^2*1i-pz^2*1i-C1^2*px^2*1i-sqrt((a2*d1*2.0-a2*pz*2.0)^2+(C1*a2*px*2.0+S1*a2*py*2.0)^2-(d1*pz*-2.0+a2^2-a3^2+d1^2+pz^2+C1^2*px^2+S1^2*py^2+C1*S1*px*py*2.0)^2)-S1^2*py^2*1i-C1*S1*px*py*2.0i);
                else
                    q(2) = -angle(a2*d1*-2.0+a2*pz*2.0-C1*a2*px*2.0i-S1*a2*py*2.0i)+angle(d1*pz*2.0i-a2^2*1i+a3^2*1i-d1^2*1i-pz^2*1i-C1^2*px^2*1i+sqrt((a2*d1*2.0-a2*pz*2.0)^2+(C1*a2*px*2.0+S1*a2*py*2.0)^2-(d1*pz*-2.0+a2^2-a3^2+d1^2+pz^2+C1^2*px^2+S1^2*py^2+C1*S1*px*py*2.0)^2)-S1^2*py^2*1i-C1*S1*px*py*2.0i);
                end
                S2 = sin(q(2));
                C2 = cos(q(2));
                
                if sol(3) == 1
                    q(3) = -angle(a2*-1i+C2*d1-C2*pz+S2*d1*1i-S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py)+angle(a3*1i-sqrt((-a2+S2*d1-S2*pz+C1*C2*px+C2*S1*py)^2+(-C2*d1+C2*pz+C1*S2*px+S1*S2*py)^2-a3^2));
                else
                    q(3) = -angle(a2*-1i+C2*d1-C2*pz+S2*d1*1i-S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py)+angle(a3*1i+sqrt((-a2+S2*d1-S2*pz+C1*C2*px+C2*S1*py)^2+(-C2*d1+C2*pz+C1*S2*px+S1*S2*py)^2-a3^2));
                end
            else
                if sol(1) == 1
                    q(1) = angle(px+py*1i);
                else
                    q(1) = angle(-px-py*1i);
                end
                S1 = sin(q(1));
                C1 = cos(q(1));
                
                if sol(2) == 1
                    q(2) = -angle(a2*d1*2.0-a2*pz*2.0-C1*a2*px*2.0i-S1*a2*py*2.0i)+angle(d1*pz*2.0i-a2^2*1i+a3^2*1i-d1^2*1i-pz^2*1i-C1^2*px^2*1i-sqrt((a2*d1*2.0-a2*pz*2.0)^2+(C1*a2*px*2.0+S1*a2*py*2.0)^2-(d1*pz*-2.0+a2^2-a3^2+d1^2+pz^2+C1^2*px^2+S1^2*py^2+C1*S1*px*py*2.0)^2)-S1^2*py^2*1i-C1*S1*px*py*2.0i);
                else
                    q(2) = -angle(a2*d1*2.0-a2*pz*2.0-C1*a2*px*2.0i-S1*a2*py*2.0i)+angle(d1*pz*2.0i-a2^2*1i+a3^2*1i-d1^2*1i-pz^2*1i-C1^2*px^2*1i+sqrt((a2*d1*2.0-a2*pz*2.0)^2+(C1*a2*px*2.0+S1*a2*py*2.0)^2-(d1*pz*-2.0+a2^2-a3^2+d1^2+pz^2+C1^2*px^2+S1^2*py^2+C1*S1*px*py*2.0)^2)-S1^2*py^2*1i-C1*S1*px*py*2.0i);
                end
                S2 = sin(q(2));
                C2 = cos(q(2));
                
                if sol(3) == 1
                    q(3) = -angle(a2*-1i-C2*d1+C2*pz-S2*d1*1i+S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py)+angle(a3*1i-sqrt((-a2-S2*d1+S2*pz+C1*C2*px+C2*S1*py)^2+(C2*d1-C2*pz+C1*S2*px+S1*S2*py)^2-a3^2));
                else
                    q(3) = -angle(a2*-1i-C2*d1+C2*pz-S2*d1*1i+S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py)+angle(a3*1i+sqrt((-a2-S2*d1+S2*pz+C1*C2*px+C2*S1*py)^2+(C2*d1-C2*pz+C1*S2*px+S1*S2*py)^2-a3^2));
                end
            end
            
            theta(1:3) = q;
            
        case'offset'
            % general case with 6 length parameters
            a1 = L(1).a;
            a2 = L(2).a;
            a3 = L(3).a;
            d1 = L(1).d;
            d2 = L(2).d;
            d3 = L(3).d;
            
            px = T(1,4); py = T(2,4); pz = T(3,4);
            
            %%% autogenerated code
            if L(1).alpha < 0
                
                if sol(1) == 1
                    q(1) = -angle(-px+py*1i)+angle(d2*1i+d3*1i-sqrt(d2*d3*-2.0-d2^2-d3^2+px^2+py^2));
                else
                    q(1) = angle(d2*1i+d3*1i+sqrt(d2*d3*-2.0-d2^2-d3^2+px^2+py^2))-angle(-px+py*1i);
                end
                S1 = sin(q(1));
                C1 = cos(q(1));
                
                if sol(2) == 1
                    q(2) = angle(d1*pz*2.0i-sqrt(d1*pz^3*4.0+d1^3*pz*4.0-a1^4-a2^4-a3^4-d1^4-py^4-pz^4-C1^4*px^4+C1^2*py^4*2.0-C1^4*py^4+a1^2*a2^2*2.0+a1^2*a3^2*2.0+a2^2*a3^2*2.0-a1^2*d1^2*2.0+a2^2*d1^2*2.0+a3^2*d1^2*2.0-a1^2*py^2*6.0-a2^2*py^2*2.0+a3^2*py^2*2.0-a1^2*pz^2*2.0+a2^2*pz^2*2.0+a3^2*pz^2*2.0-d1^2*py^2*2.0-d1^2*pz^2*6.0-py^2*pz^2*2.0+d1*py^2*pz*4.0+C1^3*a1*px^3*4.0+S1^3*a1*py^3*4.0-C1^2*a1^2*px^2*6.0+C1^2*a2^2*px^2*2.0+C1^2*a3^2*px^2*2.0+C1^2*a1^2*py^2*6.0+C1^2*a2^2*py^2*1.0e1-C1^2*a3^2*py^2*2.0-C1^4*a2^2*py^2*1.2e1+C1^6*a2^2*py^2*4.0-C1^2*d1^2*px^2*2.0+C1^2*d1^2*py^2*2.0-C1^2*px^2*py^2*6.0+C1^4*px^2*py^2*6.0-C1^2*px^2*pz^2*2.0+C1^2*py^2*pz^2*2.0+S1^6*a2^2*py^2*4.0+C1*a1^3*px*4.0+S1*a1^3*py*4.0+a1^2*d1*pz*4.0-a2^2*d1*pz*4.0-a3^2*d1*pz*4.0-C1*a1*a2^2*px*4.0-C1*a1*a3^2*px*4.0-C1*S1*px^3*py*4.0+C1*a1*d1^2*px*4.0+C1*a1*px*py^2*1.2e1+C1*a1*px*pz^2*4.0+S1*a1*a2^2*py*4.0-S1*a1*a3^2*py*4.0+S1*a1*d1^2*py*4.0+S1*a1*px^2*py*4.0+S1*a1*py*pz^2*4.0-C1*S1^3*px*py^3*4.0+C1*S1^3*px^3*py*4.0-C1^3*a1*px*py^2*1.2e1-S1^3*a1*a2^2*py*8.0+C1^2*d1*px^2*pz*4.0-C1^2*d1*py^2*pz*4.0-S1^3*a1*px^2*py*4.0-C1*a1*d1*px*pz*8.0-S1*a1*d1*py*pz*8.0-C1*S1*a1^2*px*py*1.2e1-C1*S1*a2^2*px*py*4.0+C1*S1*a3^2*px*py*4.0-C1*S1*d1^2*px*py*4.0-C1*S1*px*py*pz^2*4.0-C1^2*S1*a1*a2^2*py*8.0+C1^2*S1*a1*px^2*py*8.0+C1*S1^3*a2^2*px*py*8.0+C1^3*S1*a2^2*px*py*8.0+C1*S1*d1*px*py*pz*8.0)-a1^2*1i-a2^2*1i+a3^2*1i-d1^2*1i-py^2*1i-pz^2*1i-C1^2*px^2*1i+C1^2*py^2*1i+C1*a1*px*2.0i+S1*a1*py*2.0i-C1*S1*px*py*2.0i)-angle(-a2*(a1*-1i+d1-pz+C1*px*1i+S1^3*py*1i+C1^2*S1*py*1i));
                else
                    q(2) = angle(d1*pz*2.0i+sqrt(d1*pz^3*4.0+d1^3*pz*4.0-a1^4-a2^4-a3^4-d1^4-py^4-pz^4-C1^4*px^4+C1^2*py^4*2.0-C1^4*py^4+a1^2*a2^2*2.0+a1^2*a3^2*2.0+a2^2*a3^2*2.0-a1^2*d1^2*2.0+a2^2*d1^2*2.0+a3^2*d1^2*2.0-a1^2*py^2*6.0-a2^2*py^2*2.0+a3^2*py^2*2.0-a1^2*pz^2*2.0+a2^2*pz^2*2.0+a3^2*pz^2*2.0-d1^2*py^2*2.0-d1^2*pz^2*6.0-py^2*pz^2*2.0+d1*py^2*pz*4.0+C1^3*a1*px^3*4.0+S1^3*a1*py^3*4.0-C1^2*a1^2*px^2*6.0+C1^2*a2^2*px^2*2.0+C1^2*a3^2*px^2*2.0+C1^2*a1^2*py^2*6.0+C1^2*a2^2*py^2*1.0e1-C1^2*a3^2*py^2*2.0-C1^4*a2^2*py^2*1.2e1+C1^6*a2^2*py^2*4.0-C1^2*d1^2*px^2*2.0+C1^2*d1^2*py^2*2.0-C1^2*px^2*py^2*6.0+C1^4*px^2*py^2*6.0-C1^2*px^2*pz^2*2.0+C1^2*py^2*pz^2*2.0+S1^6*a2^2*py^2*4.0+C1*a1^3*px*4.0+S1*a1^3*py*4.0+a1^2*d1*pz*4.0-a2^2*d1*pz*4.0-a3^2*d1*pz*4.0-C1*a1*a2^2*px*4.0-C1*a1*a3^2*px*4.0-C1*S1*px^3*py*4.0+C1*a1*d1^2*px*4.0+C1*a1*px*py^2*1.2e1+C1*a1*px*pz^2*4.0+S1*a1*a2^2*py*4.0-S1*a1*a3^2*py*4.0+S1*a1*d1^2*py*4.0+S1*a1*px^2*py*4.0+S1*a1*py*pz^2*4.0-C1*S1^3*px*py^3*4.0+C1*S1^3*px^3*py*4.0-C1^3*a1*px*py^2*1.2e1-S1^3*a1*a2^2*py*8.0+C1^2*d1*px^2*pz*4.0-C1^2*d1*py^2*pz*4.0-S1^3*a1*px^2*py*4.0-C1*a1*d1*px*pz*8.0-S1*a1*d1*py*pz*8.0-C1*S1*a1^2*px*py*1.2e1-C1*S1*a2^2*px*py*4.0+C1*S1*a3^2*px*py*4.0-C1*S1*d1^2*px*py*4.0-C1*S1*px*py*pz^2*4.0-C1^2*S1*a1*a2^2*py*8.0+C1^2*S1*a1*px^2*py*8.0+C1*S1^3*a2^2*px*py*8.0+C1^3*S1*a2^2*px*py*8.0+C1*S1*d1*px*py*pz*8.0)-a1^2*1i-a2^2*1i+a3^2*1i-d1^2*1i-py^2*1i-pz^2*1i-C1^2*px^2*1i+C1^2*py^2*1i+C1*a1*px*2.0i+S1*a1*py*2.0i-C1*S1*px*py*2.0i)-angle(-a2*(a1*-1i+d1-pz+C1*px*1i+S1^3*py*1i+C1^2*S1*py*1i));
                end
                S2 = sin(q(2));
                C2 = cos(q(2));
                
                if sol(3) == 1
                    q(3) = angle(a3*1i-sqrt(d1*pz*-2.0+a1^2+a2^2-a3^2+d1^2+py^2+pz^2+C1^2*px^2-C1^2*py^2+C2*a1*a2*2.0-C1*a1*px*2.0-S2*a2*d1*2.0-S1*a1*py*2.0+S2*a2*pz*2.0-C1*C2*a2*px*2.0-C2*S1*a2*py*2.0+C1*S1*px*py*2.0))-angle(a2*-1i-C2*a1*1i+C2*d1-C2*pz+S2*a1+S2*d1*1i-S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py);
                else
                    q(3) = -angle(a2*-1i-C2*a1*1i+C2*d1-C2*pz+S2*a1+S2*d1*1i-S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py)+angle(a3*1i+sqrt(d1*pz*-2.0+a1^2+a2^2-a3^2+d1^2+py^2+pz^2+C1^2*px^2-C1^2*py^2+C2*a1*a2*2.0-C1*a1*px*2.0-S2*a2*d1*2.0-S1*a1*py*2.0+S2*a2*pz*2.0-C1*C2*a2*px*2.0-C2*S1*a2*py*2.0+C1*S1*px*py*2.0));
                end
            else
                if sol(1) == 1
                    q(1) = -angle(px-py*1i)+angle(d2*1i+d3*1i-sqrt(d2*d3*-2.0-d2^2-d3^2+px^2+py^2));
                else
                    q(1) = -angle(px-py*1i)+angle(d2*1i+d3*1i+sqrt(d2*d3*-2.0-d2^2-d3^2+px^2+py^2));
                end
                S1 = sin(q(1));
                C1 = cos(q(1));
                
                if sol(2) == 1
                    q(2) = angle(d1*pz*2.0i-sqrt(d1*pz^3*4.0+d1^3*pz*4.0-a1^4-a2^4-a3^4-d1^4-py^4-pz^4-C1^4*px^4+C1^2*py^4*2.0-C1^4*py^4+a1^2*a2^2*2.0+a1^2*a3^2*2.0+a2^2*a3^2*2.0-a1^2*d1^2*2.0+a2^2*d1^2*2.0+a3^2*d1^2*2.0-a1^2*py^2*6.0-a2^2*py^2*2.0+a3^2*py^2*2.0-a1^2*pz^2*2.0+a2^2*pz^2*2.0+a3^2*pz^2*2.0-d1^2*py^2*2.0-d1^2*pz^2*6.0-py^2*pz^2*2.0+d1*py^2*pz*4.0+C1^3*a1*px^3*4.0+S1^3*a1*py^3*4.0-C1^2*a1^2*px^2*6.0+C1^2*a2^2*px^2*2.0+C1^2*a3^2*px^2*2.0+C1^2*a1^2*py^2*6.0+C1^2*a2^2*py^2*1.0e1-C1^2*a3^2*py^2*2.0-C1^4*a2^2*py^2*1.2e1+C1^6*a2^2*py^2*4.0-C1^2*d1^2*px^2*2.0+C1^2*d1^2*py^2*2.0-C1^2*px^2*py^2*6.0+C1^4*px^2*py^2*6.0-C1^2*px^2*pz^2*2.0+C1^2*py^2*pz^2*2.0+S1^6*a2^2*py^2*4.0+C1*a1^3*px*4.0+S1*a1^3*py*4.0+a1^2*d1*pz*4.0-a2^2*d1*pz*4.0-a3^2*d1*pz*4.0-C1*a1*a2^2*px*4.0-C1*a1*a3^2*px*4.0-C1*S1*px^3*py*4.0+C1*a1*d1^2*px*4.0+C1*a1*px*py^2*1.2e1+C1*a1*px*pz^2*4.0+S1*a1*a2^2*py*4.0-S1*a1*a3^2*py*4.0+S1*a1*d1^2*py*4.0+S1*a1*px^2*py*4.0+S1*a1*py*pz^2*4.0-C1*S1^3*px*py^3*4.0+C1*S1^3*px^3*py*4.0-C1^3*a1*px*py^2*1.2e1-S1^3*a1*a2^2*py*8.0+C1^2*d1*px^2*pz*4.0-C1^2*d1*py^2*pz*4.0-S1^3*a1*px^2*py*4.0-C1*a1*d1*px*pz*8.0-S1*a1*d1*py*pz*8.0-C1*S1*a1^2*px*py*1.2e1-C1*S1*a2^2*px*py*4.0+C1*S1*a3^2*px*py*4.0-C1*S1*d1^2*px*py*4.0-C1*S1*px*py*pz^2*4.0-C1^2*S1*a1*a2^2*py*8.0+C1^2*S1*a1*px^2*py*8.0+C1*S1^3*a2^2*px*py*8.0+C1^3*S1*a2^2*px*py*8.0+C1*S1*d1*px*py*pz*8.0)-a1^2*1i-a2^2*1i+a3^2*1i-d1^2*1i-py^2*1i-pz^2*1i-C1^2*px^2*1i+C1^2*py^2*1i+C1*a1*px*2.0i+S1*a1*py*2.0i-C1*S1*px*py*2.0i)-angle(-a2*(a1*-1i-d1+pz+C1*px*1i+S1^3*py*1i+C1^2*S1*py*1i));
                else
                    q(2) = angle(d1*pz*2.0i+sqrt(d1*pz^3*4.0+d1^3*pz*4.0-a1^4-a2^4-a3^4-d1^4-py^4-pz^4-C1^4*px^4+C1^2*py^4*2.0-C1^4*py^4+a1^2*a2^2*2.0+a1^2*a3^2*2.0+a2^2*a3^2*2.0-a1^2*d1^2*2.0+a2^2*d1^2*2.0+a3^2*d1^2*2.0-a1^2*py^2*6.0-a2^2*py^2*2.0+a3^2*py^2*2.0-a1^2*pz^2*2.0+a2^2*pz^2*2.0+a3^2*pz^2*2.0-d1^2*py^2*2.0-d1^2*pz^2*6.0-py^2*pz^2*2.0+d1*py^2*pz*4.0+C1^3*a1*px^3*4.0+S1^3*a1*py^3*4.0-C1^2*a1^2*px^2*6.0+C1^2*a2^2*px^2*2.0+C1^2*a3^2*px^2*2.0+C1^2*a1^2*py^2*6.0+C1^2*a2^2*py^2*1.0e1-C1^2*a3^2*py^2*2.0-C1^4*a2^2*py^2*1.2e1+C1^6*a2^2*py^2*4.0-C1^2*d1^2*px^2*2.0+C1^2*d1^2*py^2*2.0-C1^2*px^2*py^2*6.0+C1^4*px^2*py^2*6.0-C1^2*px^2*pz^2*2.0+C1^2*py^2*pz^2*2.0+S1^6*a2^2*py^2*4.0+C1*a1^3*px*4.0+S1*a1^3*py*4.0+a1^2*d1*pz*4.0-a2^2*d1*pz*4.0-a3^2*d1*pz*4.0-C1*a1*a2^2*px*4.0-C1*a1*a3^2*px*4.0-C1*S1*px^3*py*4.0+C1*a1*d1^2*px*4.0+C1*a1*px*py^2*1.2e1+C1*a1*px*pz^2*4.0+S1*a1*a2^2*py*4.0-S1*a1*a3^2*py*4.0+S1*a1*d1^2*py*4.0+S1*a1*px^2*py*4.0+S1*a1*py*pz^2*4.0-C1*S1^3*px*py^3*4.0+C1*S1^3*px^3*py*4.0-C1^3*a1*px*py^2*1.2e1-S1^3*a1*a2^2*py*8.0+C1^2*d1*px^2*pz*4.0-C1^2*d1*py^2*pz*4.0-S1^3*a1*px^2*py*4.0-C1*a1*d1*px*pz*8.0-S1*a1*d1*py*pz*8.0-C1*S1*a1^2*px*py*1.2e1-C1*S1*a2^2*px*py*4.0+C1*S1*a3^2*px*py*4.0-C1*S1*d1^2*px*py*4.0-C1*S1*px*py*pz^2*4.0-C1^2*S1*a1*a2^2*py*8.0+C1^2*S1*a1*px^2*py*8.0+C1*S1^3*a2^2*px*py*8.0+C1^3*S1*a2^2*px*py*8.0+C1*S1*d1*px*py*pz*8.0)-a1^2*1i-a2^2*1i+a3^2*1i-d1^2*1i-py^2*1i-pz^2*1i-C1^2*px^2*1i+C1^2*py^2*1i+C1*a1*px*2.0i+S1*a1*py*2.0i-C1*S1*px*py*2.0i)-angle(-a2*(a1*-1i-d1+pz+C1*px*1i+S1^3*py*1i+C1^2*S1*py*1i));
                end
                S2 = sin(q(2));
                C2 = cos(q(2));
                
                if sol(3) == 1
                    q(3) = angle(a3*1i-sqrt(d1*pz*-2.0+a1^2+a2^2-a3^2+d1^2+py^2+pz^2+C1^2*px^2-C1^2*py^2+C2*a1*a2*2.0-C1*a1*px*2.0+S2*a2*d1*2.0-S1*a1*py*2.0-S2*a2*pz*2.0-C1*C2*a2*px*2.0-C2*S1*a2*py*2.0+C1*S1*px*py*2.0))-angle(a2*-1i-C2*a1*1i-C2*d1+C2*pz+S2*a1-S2*d1*1i+S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py);
                else
                    q(3) = -angle(a2*-1i-C2*a1*1i-C2*d1+C2*pz+S2*a1-S2*d1*1i+S2*pz*1i+C1*C2*px*1i-C1*S2*px+C2*S1*py*1i-S1*S2*py)+angle(a3*1i+sqrt(d1*pz*-2.0+a1^2+a2^2-a3^2+d1^2+py^2+pz^2+C1^2*px^2-C1^2*py^2+C2*a1*a2*2.0-C1*a1*px*2.0+S2*a2*d1*2.0-S1*a1*py*2.0-S2*a2*pz*2.0-C1*C2*a2*px*2.0-C2*S1*a2*py*2.0+C1*S1*px*py*2.0));
                end
            end
            
            theta(1:3) = q;
            
            
        case 'rrp'
            % RRP (Stanford arm like)
            
            px = T(1,4); py = T(2,4); pz = T(3,4);
            d1 = L(1).d;
            d2 = L(2).d;
            
            %%% autogenerated code
            if L(1).alpha < 0
                if sol(1) == 1
                    q(1) = -angle(-px+py*1i)+angle(d2*1i-sqrt(-d2^2+px^2+py^2));
                else
                    q(1) = angle(d2*1i+sqrt(-d2^2+px^2+py^2))-angle(-px+py*1i);
                end
                S1 = sin(q(1));
                C1 = cos(q(1));
                
                if sol(2) == 1
                    q(2) = angle(d1-pz-C1*px*1i-S1*py*1i);
                else
                    q(2) = angle(-d1+pz+C1*px*1i+S1*py*1i);
                end
                S2 = sin(q(2));
                C2 = cos(q(2));
                
                q(3) = -C2*d1+C2*pz+C1*S2*px+S1*S2*py;
                
            else
                if sol(1) == 1
                    q(1) = -angle(px-py*1i)+angle(d2*1i-sqrt(-d2^2+px^2+py^2));
                else
                    q(1) = -angle(px-py*1i)+angle(d2*1i+sqrt(-d2^2+px^2+py^2));
                end
                S1 = sin(q(1));
                C1 = cos(q(1));
                
                if sol(2) == 1
                    q(2) = angle(-d1+pz-C1*px*1i-S1*py*1i);
                else
                    q(2) = angle(d1-pz+C1*px*1i+S1*py*1i);
                end
                S2 = sin(q(2));
                C2 = cos(q(2));
                
                q(3) = -C2*d1+C2*pz-C1*S2*px-S1*S2*py;
            end
            theta(1:3) = q;
            
        case 'kr5'
            %Given function will calculate inverse kinematics for KUKA KR5 robot
            
            % Equations are calculated and implemented by
            % Gautam Sinha
            % Autobirdz Systems Pvt. Ltd.
            % SIDBI Office,
            % Indian Institute of Technology Kanpur, Kanpur, Uttar Pradesh
            % 208016
            % India
            %email- gautam.sinha705@gmail.com
            
            
            
            % get the a1, a2 and a3-- link lenghts for link no 1,2,3
            L = robot.links;
            a1 = L(1).a;
            a2 = L(2).a;
            a3 = L(3).a;
            
            % Check wether wrist is spherical or not
            if ~robot.isspherical()
                error('wrist is not spherical')
            end
            
            % get d1,d2,d3,d4---- Link offsets for link no 1,2,3,4
            d1 = L(1).d;
            d2 = L(2).d;
            d3 = L(3).d;
            d4 = L(4).d;
            
            % Get the parameters from transformation matrix
            Ox = T(1,2);
            Oy = T(2,2);
            Oz = T(3,2);
            
            Ax = T(1,3);
            Ay = T(2,3);
            Az = T(3,3);
            
            Px = T(1,4);
            Py = T(2,4);
            Pz = T(3,4);
            
            
            
            % Set the parameters n1, n2 and n3 to get required configuration from
            % solution
            n1 = -1;   % 'l'
            n2 = -1;   % 'u'
            n4 = -1;   % 'n'
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
            if ~isempty(strfind(configuration, 'n'))
                n4 = 1;
            end
            if ~isempty(strfind(configuration, 'f'))
                n4 = -1;
            end
            
            
            % Calculation for theta(1)
            r=sqrt(Px^2+Py^2);
            
            if (n1 == 1)
                theta(1)= atan2(Py,Px) + asin((d2-d3)/r);
            else
                theta(1)= atan2(Py,Px)+ pi - asin((d2-d3)/r);
            end
            
            % Calculation for theta(2)
            X= Px*cos(theta(1)) + Py*sin(theta(1)) - a1;
            r=sqrt(X^2 + (Pz-d1)^2);
            Psi = acos((a2^2-d4^2-a3^2+X^2+(Pz-d1)^2)/(2.0*a2*r));
            
            if ~isreal(Psi)
                warning('RTB:ikine6s:notreachable', 'point not reachable');
                theta = [NaN NaN NaN NaN NaN NaN];
                return
            end
            
            theta(2) = atan2((Pz-d1),X) + n2*Psi;
            
            % Calculation for theta(3)
            Nu = cos(theta(2))*X + sin(theta(2))*(Pz-d1) - a2;
            Du = sin(theta(2))*X - cos(theta(2))*(Pz-d1);
            theta(3) = atan2(a3,d4) - atan2(Nu, Du);
            
            % Calculation for theta(4)
            Y = cos(theta(1))*Ax + sin(theta(1))*Ay;
            M2 = sin(theta(1))*Ax - cos(theta(1))*Ay ;
            M1 =  ( cos(theta(2)-theta(3)) )*Y + ( sin(theta(2)-theta(3)) )*Az;
            theta(4) = atan2(n4*M2,n4*M1);
            
            % Calculation for theta(5)
            Nu =  -cos(theta(4))*M1 - M2*sin(theta(4));
            M3 =  -Az*( cos(theta(2)-theta(3)) ) + Y*( sin(theta(2)-theta(3)) );
            theta(5) = atan2(Nu,M3);
            
            % Calculation for theta(6)
            Z = cos(theta(1))*Ox + sin(theta(1))*Oy;
            L2 = sin(theta(1))*Ox - cos(theta(1))*Oy;
            L1 = Z*( cos(theta(2)-theta(3) )) + Oz*( sin(theta(2)-theta(3)));
            L3 = Z*( sin(theta(2)-theta(3) )) - Oz*( cos(theta(2)-theta(3)));
            A1 = L1*cos(theta(4)) + L2*sin(theta(4));
            A3 = L1*sin(theta(4)) - L2*cos(theta(4));
            Nu =  -A1*cos(theta(5)) - L3*sin(theta(5));
            Du =  -A3;
            theta(6) = atan2(Nu,Du);
            

        otherwise
            error('RTB:ikine6s:badarg', 'Unknown solution type [%s]', robot.ikineType);
    end
    % Solve for the wrist rotation
    
    % we need to account for some random translations between the first and last 3
    % joints (d4) and also d6,a6,alpha6 in the final frame.
    
    T13 = robot.A(1:3, theta(1:3));  % transform of first 3 joints
    

    % T = T13 * Tz(d4) * R * Tz(d6) Tx(a5)
    Td4 = transl(0, 0, L(4).d);      % Tz(d4)
    Tt = transl(L(6).a, 0, L(6).d) * trotx(L(6).alpha);  % Tz(d6) Tx(a5) Rx(alpha6)
    
    R = inv(Td4) * inv(T13) * T * inv(Tt);


    % the spherical wrist implements Euler angles
    
if sol(3) == 1
        theta(4:6) = tr2eul(R, 'flip');
    else
        theta(4:6) = tr2eul(R);
        
    end
    if L(4).alpha > 0
        theta(5) = -theta(5);
    end
    
    % remove the link offset angles
    for i=1:robot.n   %#ok<*AGROW>
        theta(i) = theta(i) - L(i).offset;
    end
end

% predicates to determine which kinematic solution to use
function s = is_simple(L)
    alpha = [-pi/2 0 pi/2];
    s =     all([L(2:3).d] == 0) && ...
            (all([L(1:3).alpha] == alpha) || all([L(1:3).alpha] == -alpha)) && ...
            all([L(1:3).sigma] == 0) && ...
            (L(1).a == 0);
end

function s = is_offset(L)
    alpha = [-pi/2 0 pi/2];
    s =     (all([L(1:3).alpha] == alpha) || all([L(1:3).alpha] == -alpha)) && ...
            all([L(1:3).sigma] == 0);
end

function s = is_rrp(L)
    alpha = [-pi/2 pi/2 0];
    s =     all([L(2:3).a] == 0) && ...
            (all([L(1:3).alpha] == alpha) || all([L(1:3).alpha] == -alpha)) && ...
            all([L(1:3).sigma] == [0 0 1]);
end

function s = is_puma(L)
    alpha = [pi/2 0 -pi/2];
    s =     (L(2).d == 0) && (L(1).a == 0) && ...
            (L(3).d ~= 0) && (L(3).a ~= 0) && ...
            all([L(1:3).alpha] == alpha) && ...
            all([L(1:3).sigma] == 0);
end
