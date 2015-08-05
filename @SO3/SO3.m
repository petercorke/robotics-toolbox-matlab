classdef SO3
    properties
        data
    end
    
    properties (Dependent=true)
        R
    end
    
    methods
        
        function obj = SO3(a, varargin)
            if nargin == 0
                obj.data = eye(3,3);
            elseif SO3.isa(a)
                obj.data = a;
            else
            end
        end
        
                function out = mtimes(obj, a)
            if isa(a, 'SO3')
                out = SO3( obj.data * a.data);
            elseif SO3.isa(a)
                out = SO3( obj.data * a);
                
            elseif isvec(a,3)
                out = obj.data * a;
            else
                error('bad thing');
            end
        end
        
                function ir = inv(obj)
            ir = SO2(obj.R');
        end
        
        function d = det(obj)
            d = det(obj.R);
        end
        
        function varargout = eig(obj, varargin)
            [varargout{1:nargout}] = eig(obj.data, varargin{:});
        end
        
        function R = get.R(obj)
            R = obj.data(1:3,1:3);
        end
        
        function o = set.R(obj, data)
            obj.data(1:3,1:3) = data;
            o = obj;
        end
        %TR2RPY Convert a homogeneous transform to roll-pitch-yaw angles
%
% RPY = TR2RPY(T, options) are the roll-pitch-yaw angles (1x3)
% corresponding to the rotation part of a homogeneous transform T. The 3
% angles RPY=[R,P,Y] correspond to sequential rotations about the X, Y and
% Z axes respectively.
%
% RPY = TR2RPY(R, options) as above but the input is an orthonormal
% rotation matrix R (3x3).
%
% If R (3x3xK) or T (4x4xK) represent a sequence then each row of RPY
% corresponds to a step of the sequence.
%
% Options::
%  'deg'   Compute angles in degrees (radians default)
%  'xyz'   Return solution for sequential rotations about X, Y, Z axes
%
% Notes::
% - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
%   set to zero and Y is the sum (R+Y).
% - Toolbox rel 8-9 has the reverse angle sequence
%
% See also  rpy2tr, tr2eul.




% TODO singularity for XYZ case, 
function rpy = torpy(obj, varargin)
	m = obj.R;
    opt.deg = false;
    opt.xyz = false;
    opt = tb_optparse(opt, varargin);

	s = size(m);
	if length(s) > 2
		rpy = zeros(s(3), 3);
		for i=1:s(3)
			rpy(i,:) = tr2rpy(m(:,:,i), varargin{:});
		end
		return
	end
	rpy = zeros(1,3);

    if opt.xyz
        % XYZ order
        if abs(m(3,3)) < eps && abs(m(2,3)) < eps
            % singularity
            rpy(1) = 0;  % roll is zero
            rpy(2) = atan2(m(1,3), m(3,3));  % pitch
            rpy(3) = atan2(m(2,1), m(2,2));  % yaw is sum of roll+yaw
        else
            rpy(1) = atan2(-m(2,3), m(3,3));        % roll
            % compute sin/cos of roll angle
            sr = sin(rpy(1));
            cr = cos(rpy(1));
            rpy(2) = atan2(m(1,3), cr * m(3,3) - sr * m(2,3));  % pitch
            rpy(3) = atan2(-m(1,2), m(1,1));        % yaw
        end
    else
        % old ZYX order (as per Paul book)
        if abs(m(1,1)) < eps && abs(m(2,1)) < eps
            % singularity
            rpy(1) = 0;     % roll is zero
            rpy(2) = atan2(-m(3,1), m(1,1));  % pitch
            rpy(3) = atan2(-m(2,3), m(2,2));  % yaw is difference yaw-roll
        else
            rpy(1) = atan2(m(2,1), m(1,1));
            sp = sin(rpy(1));
            cp = cos(rpy(1));
            rpy(2) = atan2(-m(3,1), cp * m(1,1) + sp * m(2,1));
            rpy(3) = atan2(sp * m(1,3) - cp * m(2,3), cp*m(2,2) - sp*m(1,2));
        end
    end
    if opt.deg
        rpy = rpy * 180/pi;
    end
end
%TR2EUL Convert homogeneous transform to Euler angles
%
% EUL = TR2EUL(T, OPTIONS) are the ZYZ Euler angles (1x3) corresponding to
% the rotational part of a homogeneous transform T (4x4). The 3 angles
% EUL=[PHI,THETA,PSI] correspond to sequential rotations about the Z, Y and
% Z axes respectively.
%
% EUL = TR2EUL(R, OPTIONS) as above but the input is an orthonormal
% rotation matrix R (3x3).
%
% If R (3x3xK) or T (4x4xK) represent a sequence then each row of EUL
% corresponds to a step of the sequence.
%
% Options::
%  'deg'      Compute angles in degrees (radians default)
%  'flip'     Choose first Euler angle to be in quadrant 2 or 3.
%
% Notes::
% - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
%   set to zero and PSI is the sum (PHI+PSI).
%
% See also  EUL2TR, TR2RPY.




function euler = toeul(obj, varargin)
    
    R = obj.R;

    opt.deg = false;
    opt.flip = false;
    opt = tb_optparse(opt, varargin);

	s = size(R);
	if length(s) > 2
        euler = zeros(s(3), 3);
		for i=1:s(3)
			euler(i,:) = tr2eul(R(:,:,i));
		end

        if opt.deg
            euler = euler * 180/pi;
        end
		return
	end

	euler = zeros(1,3);

	% Method as per Paul, p 69.
    % euler = [phi theta psi]
    %

	if abs(R(1,3)) < eps && abs(R(2,3)) < eps
		% singularity
		euler(1) = 0;
		sp = 0;
		cp = 1;
		euler(2) = atan2(cp*R(1,3) + sp*R(2,3), R(3,3));
		euler(3) = atan2(-sp * R(1,1) + cp * R(2,1), -sp*R(1,2) + cp*R(2,2));
    else
        % non singular
        
        % Only positive phi is returned.
        if opt.flip
            euler(1) = atan2(-R(2,3), -R(1,3));
        else
            euler(1) = atan2(R(2,3), R(1,3));
        end
		sp = sin(euler(1));
		cp = cos(euler(1));
		euler(2) = atan2(cp*R(1,3) + sp*R(2,3), R(3,3));
		euler(3) = atan2(-sp * R(1,1) + cp * R(2,1), -sp*R(1,2) + cp*R(2,2));
	end
    if opt.deg
        euler = euler * 180/pi;
    end
end

        
        %TR2ANGVEC Convert rotation matrix to angle-vector form
%
% [THETA,V] = TR2ANGVEC(R, OPTIONS) is rotation expressed in terms of an
% angle THETA (1x1) about the axis V (1x3) equivalent to the orthonormal rotation
% matrix R (3x3).
%
% [THETA,V] = TR2ANGVEC(T, OPTIONS) as above but uses the rotational part of the
% homogeneous transform T (4x4).
%
% If R (3x3xK) or T (4x4xK) represent a sequence then THETA (Kx1)is a vector 
% of angles for corresponding elements of the sequence and V (Kx3) are the 
% corresponding axes, one per row.
%
% Options::
% 'deg'   Return angle in degrees
%
% Notes::
% - If no output arguments are specified the result is displayed.
%
% See also ANGVEC2R, ANGVEC2TR, TRLOG.


function [theta_, n_] = toangvec(obj, varargin)
    R = obj.R;

    opt.deg = false;
    opt = tb_optparse(opt, varargin);
    
    % get the rotation submatrix(s)
    if ~isrot(R)
        R = t2r(R);
    end
    
    if size(R,3) > 1
        theta = zeros(size(R,3),1);
        v = zeros(size(R,3),3);
    end
    
    for i=1:size(R,3)  % for each rotation matrix in the sequence
        
        % There are a few ways to do this:
        %
        % 1.
        %
        % e = 0.5*vex(R - R');  % R-R' is skew symmetric
        % theta = asin(norm(e));
        % n = unit(e);
        %
        %  but this fails for rotations > pi/2
        %
        % 2.
        %
        % e = vex(logm(R));
        % theta = norm(e);
        % n = unit(e);
        %
        %  elegant, but 40x slower than using eig
        %
        % 3.
        %
        % Use eigenvectors, get angle from trace which is defined over -pi to
        % pi.  Don't use eigenvalues since they only give angles -pi/2 to pi/2.
        %
        % 4.
        %
        % Take the log of the rotation matrix
        
        Ri = R(:,:,i);
        
        % check the determinant
        if abs(det(Ri)-1) > 10*eps
            error('RTB:tr2angvec:badarg', 'matrix is not orthonormal');
        end
        
        [th,v] = trlog(Ri);
        theta(i) = th;
        n(i,:) = v;
        
        if opt.deg
            theta(i) = theta(i) * 180/pi;
            units = 'deg';
        else
            units = 'rad';
        end
        
        if nargout == 0
            % if no output arguments display the angle and vector
            fprintf('Rotation: %f %s x [%f %f %f]\n', theta(i), units, n(i,:));
        end
    end
    
    if nargout == 1
        theta_ = theta;
    elseif nargout == 2
        theta_ = theta;
        n_ = n;
    end
end
        
        function display(l)
            %Link.display Display parameters
            %
            % L.display() displays the link parameters in compact single line format.  If L is a
            % vector of Link objects displays one line per element.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Link object and the command has no trailing
            %   semicolon.
            %
            % See also Link.char, Link.dyn, SerialLink.showlink.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(l) );
        end % display()
        
        
        function s2 = char(obj)
            s = num2str(obj.data, '%10.4g'); %num2str(obj.data, 4);
            for i=1:numrows(s);
                s2(i,:) = ['    ', s(i,:)];
            end
        end
    end
    
    methods (Static)
        
        %ROTX Rotation about X axis
        %
        % R = ROTX(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA
        % radians about the x-axis.
        %
        % R = ROTX(THETA, 'deg') as above but THETA is in degrees.
        %
        % See also ROTY, ROTZ, ANGVEC2R, ROT2.
        
        
        function obj = rotx(t, deg)
            
            if nargin > 1 && strcmp(deg, 'deg')
                t = t *pi/180;
            end
            
            ct = cos(t);
            st = sin(t);
            R = [
                1   0    0
                0   ct  -st
                0   st   ct
                ];
            obj = SO3(R);
        end
        %ROTY Rotation about Y axis
        %
        % R = ROTY(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA
        % radians about the y-axis.
        %
        % R = ROTY(THETA, 'deg') as above but THETA is in degrees.
        %
        % See also ROTX, ROTZ, ANGVEC2R, ROT2.
        
        
        
        function obj = roty(t, deg)
            if nargin > 1 && strcmp(deg, 'deg')
                t = t *pi/180;
            end
            ct = cos(t);
            st = sin(t);
            R = [
                ct  0   st
                0   1   0
                -st  0   ct
                ];
            obj = SO3(R);
        end
        %ROTZ Rotation about Z axis
        %
        % R = ROTZ(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA
        % radians about the z-axis.
        %
        % R = ROTZ(THETA, 'deg') as above but THETA is in degrees.
        %
        % See also ROTX, ROTY, ANGVEC2R, ROT2.
        
        
        
        function obj = rotz(t, deg)
            if nargin > 1 && strcmp(deg, 'deg')
                t = t *pi/180;
            end
            
            ct = cos(t);
            st = sin(t);
            R = [
                ct  -st  0
                st   ct  0
                0    0   1
                ];
            obj = SO3(R);
        end
        %ISROT Test if SO(3) rotation matrix
        %
        % ISROT(R) is true (1) if the argument is of dimension 3x3 or 3x3xN, else false (0).
        %
        % ISROT(R, 'valid') as above, but also checks the validity of the rotation
        % matrix.
        %
        % Notes::
        % - A valid rotation matrix has determinant of 1.
        %
        % See also ISHOMOG, ISVEC.
        
        function h = isa(r, dtest)
            
            d = size(r);
            if ndims(r) >= 2
                h =  all(d(1:2) == [3 3]);
                
                if h && nargin > 1
                    h = abs(det(r) - 1) < eps;
                end
                
            else
                h = false;
            end
        end
        
        %EUL2R Convert Euler angles to rotation matrix
        %
        % R = EUL2R(PHI, THETA, PSI, OPTIONS) is an SO(2) orthonornal rotation
        % matrix (3x3) equivalent to the specified Euler angles.  These correspond
        % to rotations about the Z, Y, Z axes respectively. If PHI, THETA, PSI are
        % column vectors (Nx1) then they are assumed to represent a trajectory and
        % R is a three-dimensional matrix (3x3xN), where the last index corresponds
        % to rows of PHI, THETA, PSI.
        %
        % R = EUL2R(EUL, OPTIONS) as above but the Euler angles are taken from
        % consecutive columns of the passed matrix EUL = [PHI THETA PSI].  If EUL
        % is a matrix (Nx3) then they are assumed to represent a trajectory and R
        % is a three-dimensional matrix (3x3xN), where the last index corresponds
        % to rows of EUL which are assumed to be [PHI, THETA, PSI].
        %
        % Options::
        %  'deg'      Compute angles in degrees (radians default)
        %
        % Note::
        % - The vectors PHI, THETA, PSI must be of the same length.
        %
        % See also EUL2TR, RPY2TR, TR2EUL.
        
        
        
        function obj = eul(phi, varargin)
            opt.deg = false;
            [opt,args] = tb_optparse(opt, varargin);
            
            % unpack the arguments
            if numcols(phi) == 3
                theta = phi(:,2);
                psi = phi(:,3);
                phi = phi(:,1);
            elseif nargin >= 3
                theta = args{1};
                psi = args{2};
            else
                error('RTB:eul2r:badarg', 'expecting 3 inputs, 3-vector or 3-column matrix')
            end
            
            % optionally convert from degrees
            if opt.deg
                d2r = pi/180.0;
                phi = phi * d2r;
                theta = theta * d2r;
                psi = psi * d2r;
            end
            
            if numrows(phi) == 1
                R = rotz(phi) * roty(theta) * rotz(psi);
            else
                R = zeros(3,3,numrows(phi));
                for i=1:numrows(phi)
                    R(:,:,i) = rotz(phi(i)) * roty(theta(i)) * rotz(psi(i));
                end
            end
            obj = SO3(R);
        end
        %OA2R Convert orientation and approach vectors to rotation matrix
        %
        % R = OA2R(O, A) is an SO(3) rotation matrix (3x3) for the specified
        % orientation and approach vectors (3x1) formed from 3 vectors such that R
        % = [N O A] and N = O x A.
        %
        % Notes::
        % - The submatrix is guaranteed to be orthonormal so long as O and A
        %   are not parallel.
        % - The vectors O and A are parallel to the Y- and Z-axes of the coordinate
        %   frame.
        %
        % References::
        % - Robot manipulators: mathematis, programming and control
        %   Richard Paul, MIT Press, 1981.
        %
        % See also RPY2R, EUL2R, OA2TR.
        
        
        
        function obj = oa(o, a)
            
            if nargin < 2 || ~isvec(o) || ~isvec(a)
                error('RTB:oa2r:badarg', 'bad arguments');
            end
            
            o = o(:); a = a(:);
            n = cross(o, a);
            o = cross(a, n);
            R = [unit(n(:)) unit(o(:)) unit(a(:))];
            obj = SO3(R);
        end
        %RPY2R Roll-pitch-yaw angles to rotation matrix
        %
        % R = RPY2R(ROLL, PITCH, YAW, OPTIONS) is an SO(3) orthonornal rotation
        % matrix (3x3) equivalent to the specified roll, pitch, yaw angles angles.
        % These correspond to rotations about the X, Y, Z axes respectively. If
        % ROLL, PITCH, YAW are column vectors (Nx1) then they are assumed to
        % represent a trajectory and R is a three-dimensional matrix (3x3xN), where
        % the last index corresponds to rows of ROLL, PITCH, YAW.
        %
        % R = RPY2R(RPY, OPTIONS) as above but the roll, pitch, yaw angles angles
        % angles are taken from consecutive columns of the passed matrix RPY =
        % [ROLL, PITCH, YAW].  If RPY is a matrix (Nx3) then they are assumed to
        % represent a trajectory and R is a three-dimensional matrix (3x3xN), where
        % the last index corresponds to rows of RPY which are assumed to be [ROLL,
        % PITCH, YAW].
        %
        % Options::
        %  'deg'   Compute angles in degrees (radians default)
        %  'xyz'   Return solution for sequential rotations about X, Y, Z axes
        %
        % Note::
        % - Toolbox rel 8-9 has the reverse angle sequence
        %
        % See also TR2RPY, EUL2TR.
        
        
        
        function obj = rpy(roll, varargin)
            opt.xyz = false;
            opt.deg = false;
            [opt,args] = tb_optparse(opt, varargin);
            
            % unpack the arguments
            if numcols(roll) == 3
                pitch = roll(:,2);
                yaw = roll(:,3);
                roll = roll(:,1);
            elseif nargin >= 3
                pitch = args{1};
                yaw = args{2};
            else
                error('RTB:rpy2r:badarg', 'bad arguments')
            end
            
            % optionally convert from degrees
            if opt.deg
                d2r = pi/180.0;
                roll = roll * d2r;
                pitch = pitch * d2r;
                yaw = yaw * d2r;
            end
            
            if opt.xyz
                % XYZ order
                if numrows(roll) == 1
                    R = rotx(roll) * roty(pitch) * rotz(yaw);
                else
                    R = zeros(3,3,numrows(roll));
                    for i=1:numrows(roll)
                        R(:,:,i) = rotx(roll(i)) * roty(pitch(i)) * rotz(yaw(i));
                    end
                end
            else
                % old ZYX order (as per Paul book)
                if numrows(roll) == 1
                    R = rotz(roll) * roty(pitch) * rotx(yaw);
                else
                    R = zeros(3,3,numrows(roll));
                    for i=1:numrows(roll)
                        R(:,:,i) = rotz(roll(i)) * roty(pitch(i)) * rotx(yaw(i));
                    end
                end
            end
            obj = SO3(R);
        end
        
        function obj = angvec(theta, k)
            %ANGVEC2R Convert angle and vector orientation to a rotation matrix
            %
            % R = ANGVEC2R(THETA, V) is an orthonormal rotation matrix (3x3)
            % equivalent to a rotation of THETA about the vector V.
            %
            % Notes::
            % - If THETA == 0 then return identity matrix.
            % - If THETA ~= 0 then V must have a finite length.
            %
            % See also eul2r, rpy2r, tr2angvec.
            
            if nargin < 2 || ~isscalar(theta) || ~isvec(k)
                error('RTB:angvec2r:badarg', 'bad arguments');
            end
            if norm(k) < 10*eps
                if (abs(theta) > 0)
                    error('RTB:angvec2r:badarg', 'norm of direction is zero');
                else
                    R = eye(3,3);
                    return;
                end
            end
            obj = SO3(R);
        end
    end
end