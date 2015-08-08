%UnitQuaternion Unit-quaternion class
%
% A unit-quaternion is a compact method of representing a 3D rotation that has
% computational advantages including speed and numerical robustness.
% A quaternion has 2 parts, a scalar s, and a vector v and is typically
% written: q = s <vx, vy, vz>.
%
% A unit-quaternion is one for which s^2+vx^2+vy^2+vz^2 = 1.  It can be
% considered as a rotation by an angle theta about a unit-vector V in space where
%
%         q = cos (theta/2) < v sin(theta/2)>
%
%
% Methods::
%  UnitQuaternion          constructor
%  UnitQuaternion.eul      constructor, from Euler angles
%  UnitQuaternion.rpy      constructor, from roll-pitch-yaw angles
%  UnitQuaternion.angvec   constructor, from (angle vector)
%  UnitQuaternion.omega    constructor for angle*vector
%  UnitQuaternion.Rx       constructor, from x-axis rotation
%  UnitQuaternion.Ry       constructor, from y-axis rotation
%  UnitQuaternion.Rz       constructor, from z-axis rotation
%  inv                     inverse
%  conj                    conjugate
%  unit                    unitized quaternion
%  dot                     derivative of quaternion with angular velocity w
%  interp                  interpolation (slerp) between quaternions
%  norm                    norm, or length
%  inner                   inner product
%  angle                   angle between two quaternions
%  plot                    plot a coordinate frame representing orientation of quaternion
%  animate                 animates a coordinate frame representing changing orientation
%                          of quaternion sequence
% Conversion methods::
%  R                       convert to 3x3 rotation matrix
%  T                       convert to 4x4 homogeneous transform matrix
%  toeul                   convert to Euler angles
%  torpy                   convert to roll-pitch-yaw angles
%  toangvec                convert to angle vector form
%  SO3                     convert to SO3 class
%  SE3                     convert to SE3 class
%  double                  quaternion elements as 4-vector
%
% Overloaded operators::
%  q*q2                    quaternion (Hamilton) product
%  q.*q2                   quaternion (Hamilton) product followed by unitization
%  q/q2                    q*q2.inv
%  q./q2                   q*q2.inv followed by unitization
%  q^n                     q to power n (integer only)
%  q1==q2                  test for quaternion equality
%  q1~=q2                  test for quaternion inequality
%
%
%
% Properties (read only)::
%  s         real part
%  v         vector part
%
% Notes::
% - UnitQuaternion objects can be used in vectors and arrays.
%
% References::
% - Animating rotation with quaternion curves,
%   K. Shoemake,
%   in Proceedings of ACM SIGGRAPH, (San Fran cisco), pp. 245-254, 1985.
% - On homogeneous transforms, quaternions, and computational efficiency,
%   J. Funda, R. Taylor, and R. Paul,
%   IEEE Transactions on Robotics and Automation, vol. 6, pp. 382-388, June 1990.
% - Robotics, Vision & Control,
%   P. Corke, Springer 2011.
%
% See also Quaternion, SO3, SE3.


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

% TODO
%  constructor handles R, T trajectory and returns vector
%  .r, .t on a quaternion vector??

classdef UnitQuaternion < Quaternion
    
    methods
        
        function uq = UnitQuaternion(s, v)
            %UnitQuaternion.Quaternion Constructor for quaternion objects
            %
            % Construct a unit-quaternion from various other orientation representations.
            %
            % Q = UnitQuaternion() is the identitity unit-quaternion 1<0,0,0> representing a null rotation.
            %
            % Q = UnitQuaternion(Q1) is a copy of the unit-quaternion Q1, if Q1 is a
            % Quaternion it is normalised.
            %
            % Q = UnitQuaternion(S, V) is a unit quaternion formed by specifying directly
            % its scalar and vector parts which are normalised.
            %
            % Q = UnitQuaternion([S V1 V2 V3]) is a quaternion formed by specifying
            % directly its 4 elements which are normalised.
            %
            % Q = Quaternion(R) is a unit-quaternion corresponding to the SO(3)
            % orthonormal rotation matrix R (3x3).  If R (3x3xN) is a sequence then Q
            % (Nx1) is a vector of Quaternions corresponding to the elements of R.
            %
            % Q = Quaternion(T) is a unit-quaternion equivalent to the rotational part
            % of the SE(3) homogeneous transform T (4x4). If T (4x4xN) is a sequence
            % then Q (Nx1) is a vector of Quaternions corresponding to the elements of
            % T.
            %
            % Notes::
            % - Only the R and T forms are vectorised.
            %
            % See also UnitQuaternion.eul, UnitQuaternion.rpy, UnitQuaternion.angvec, UnitQuaternion.omega, UnitQuaternion.Rx.
            if nargin == 0
                % null constructor
                %             uq.v = [0,0,0];
                uq.s = 1;
            elseif isa(s, 'Quaternion')
                % passed a quaternion of some kind, optionally normalize
                if ~isa(s, 'UnitQuaternion');
                    s = s.unit();
                end
                uq.s = s.s;
                uq.v = s.v;
            elseif isrot(s) || ishomog(s)
                %   Q = Quaternion(R)       from a 3x3 or 4x4 matrix
                uq(size(s,3)) = UnitQuaternion();  % preallocate
                for i=1:size(s,3)
                    [qs,qv] = UnitQuaternion.tr2q(s(:,:,i));
                    uq(i) = UnitQuaternion(qs,qv);
                end
            elseif nargin ==2 && isscalar(s) && isvec(v,3)
                % ensure its a unit quaternion
                n = norm([s; v(:)]);
                uq.s = s/n;
                uq.v = v/n;
            elseif isvec(s,4)
                % passed in a 4-vector of components
                % normalize it
                s = unit(s);
                uq.s = s(1);
                uq.v = s(2:4);
            else
                error ('RTB:UnitQuaternion:badarg', 'bad argument to quaternion constructor');
            end
            
        end
        
        % Helper function for an object that invokes it's constructor.  If invoked
        % on a UnitQuaternion in the superclass will construct a UnitQuaternion.
        function uq = new(q, varargin)
            uq = UnitQuaternion(varargin{:});
        end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% UNIT-QUATERNION FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

        function qi = inv(q)
            %UnitQuaternion.inv Invert a unit-quaternion
            %
            % QI = Q.inv() is a unit-quaternion object representing the inverse of Q.
            %
            % Notes::
            % - Supports quaternion vector.
            
            for i=1:length(q)
                qi(i) = UnitQuaternion(q(i).s, -q(i).v);
            end
        end
        
        function q = interp(Q1, varargin)
            %UnitQuaternion.interp Interpolate unit-quaternions
            %
            % QI = Q.scale(S, OPTIONS) is a unit-quaternion that interpolates between a null
            % rotation (identity quaternion) for S=0 to Q for S=1.
            %
            % QI = Q.interp(Q2, S, OPTIONS) as above but interpolates a rotation
            % between Q for S=0 and Q2 for S=1.
            %
            % If S is a vector QI is a vector of unit-quaternions, each element
            % corresponding to sequential elements of S.
            %
            % Options::
            % 'shortest'   Take the shortest path along the great circle
            %
            % Notes::
            % - This is a spherical linear interpolation (slerp) that can be interpretted
            %   as interpolation along a great circle arc on a sphere.
            % - It is an error if S is outside the interval 0 to 1.
            %
            % References::
            % - Animating rotation with quaternion curves,
            %   K. Shoemake,
            %   in Proceedings of ACM SIGGRAPH, (San Fran cisco), pp. 245-254, 1985.
            %
            % See also ctraj.
            
            opt.shortest = false;

            [opt,args] = tb_optparse(opt, varargin);
            
            if isa(args{1}, 'UnitQuaternion')
                q1 = double(Q1);
                q2 = double(args{1});
                r = args{2};
            else
                q1 = [1  0 0 0];
                q2 = double(Q1);
                r = args{1};
            end
            
            cosTheta = q1*q2';
            
            if opt.shortest
                % take shortest path along the great circle, patch by Gauthier Gras
                if cosTheta < 0
                    q1 = - q1;
                    cosTheta = - cosTheta;
                end;
            end
            
            theta = acos(cosTheta);            

            if any(r<0 | r>1)
                error('RTB:UnitQuaternion:interp', 'values of S outside interval [0,1]');
            end
            
            q(length(r)) = UnitQuaternion();  % preallocate space for Quaternion vector
            
            for i=1:length(r)
                if theta == 0
                    q(i) = Q1;
                else
                    q(i) = UnitQuaternion( (sin((1-r(i))*theta) * q1 + sin(r(i)*theta) * q2) / sin(theta) );
                end
            end
        end
        
        
        function th = angle(q1, q2)
            %UnitQuaternion.angle Angle between two unit-quaternions
            %
            % Q.theta(Q2) is the angle (in radians) between two unit-quaternions Q and Q2.
            %
            % Notes::
            % - Either or both Q and Q2 can be a vector.
            %
            % See also Quaternion.angvec.
            
            c = zeros(1, max(length(q1), length(q2)));
            if length(q1) == length(q2)
                for i=1:length(q1)
                    c(i) = q1(i).inner(q2(i));
                end
            elseif length(q1) == 1
                for i=1:length(q2)
                    c(i) = q1.inner(q2(i));
                end
            elseif length(q2) == 1
                for i=1:length(q1)
                    c(i) = q1(i).inner(q2);
                end
            else
                error('RTB:Quaternion:badarg', 'angle arguments must be vectors of same length');
            end
            % clip it, just in case it's unnormalized
            c(c<-1) = -1;  c(c>1) = 1;
            
            th = 2*acos(c);
        end
        
        function [theta_,n_] = toangvec(q, varargin)
            %UnitQuaternion.angvec Convert to angle-vector form
            %
            % Q.angvec(OPTIONS) prints a compact single line representation of the rotational angle and rotation vector
            % corresponding to this quaternion.
            %
            % TH = Q.angvec(OPTIONS) is the rotational angle, about some vector,
            % corresponding to this quaternion.
            %
            % [TH,V] = Q.angvec(OPTIONS) as above but also returns a unit vector
            % parallel to the rotation axis.
            %
            % Options::
            %  'deg'     Display/return angle in degrees rather than radians
            %
            % Notes::
            % - Due to the double cover of the quaternion, the returned rotation angles
            %   will be in the interval [-2pi, 2pi).
            % - If Q is a unit-quaternion vector then print one line per element.
            % - If Q is a unit-quaternion vector (1xN) then TH (1xN) and V (Nx3).
            
            opt.deg = false;
            opt = tb_optparse(opt, varargin);
            
            if opt.deg
                theta = theta * 180/pi;
                units = 'deg';
            else
                units = 'rad';
            end
            
            % compute the angle and vector for each quaternion
            for i=1:length(q)
                qi = q(i);
                if norm(qi.v) < 10*eps
                    % identity quaternion, null rotation
                    theta(i) = 0;
                    n(i,:) = [0 0 0];
                else
                    % finite rotation
                    n(i,:) = unit(qi.v);
                    theta(i) = 2*atan2( norm(qi.v), qi.s);
                end
            end
            
            % optionally display them
            if nargout == 0
                % if no output arguments display the angle and vector
                for i=1:length(q)
                    fprintf('Rotation: %f %s x [%f %f %f]\n', theta(i), units, n(i,:));
                end
            elseif nargout == 1
                theta_ = theta;
            elseif nargout == 2
                theta_ = theta;
                n_ = n;
            end
        end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% ARITHMETIC OPERATORS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

        function qp = mtimes(q1, q2)
            %UnitQuaternion.mtimes Multiply a quaternion object
            %
            % Q1*Q2   is a quaternion formed by the Hamilton product of two quaternions.
            % Q*V     is a vector formed by rotating the vector V by the quaternion Q.
            %
            % Notes::
            % - Overloaded operator '*'
            % - For case Q1*Q2 both can be an N-vector, result is elementwise
            %   multiplication.
            % - For case Q1*Q2 if Q1 scalar and Q2 a vector, scalar multiplies each
            %   element.
            % - For case Q1*Q2 if Q2 scalar and Q1 a vector, each element multiplies
            %   scalar.
            % - If the two multiplicands are unit-quaternions, the product will be a
            %   unit quaternion.
            %
            % See also Quaternion.mrdivide, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
            
            if isa(q2, 'UnitQuaternion')
                %QQMUL  Multiply unit-quaternion by unit-quaternion
                %
                %   QQ = qqmul(Q1, Q2)
                %
                %   Return a product of unit-quaternions.
                %
                %   See also: TR2Q
                
                % get the superclass to do this for us
                qp = mtimes@Quaternion(q1, q2);
                
            elseif isa(q2, 'Quaternion')
                % get the superclass to do this for us
                qp = mtimes@Quaternion(q1, q2);
                
            elseif isa(q1, 'Quaternion') && isa(q2, 'double')
                
                %QVMUL  Multiply vector by unit-quaternion
                %
                %   VT = qvmul(Q, V)
                %
                %   Rotate the vector V by the unit-quaternion Q.
                %
                %   See also: QQMUL, QINV
                qp = zeros(3, max(length(q1), numcols(q2)));
                if numrows(q2) == 3
                    if length(q1) == numcols(q2)
                        for i=1:length(q1)
                            q = q1(i) * Quaternion.pure(q2(:,i)) * inv(q1(i));
                            qp(:,i) = q.v(:);
                        end
                    elseif length(q1) == 1
                        for i=1:numcols(q2)
                            q = q1 * Quaternion.pure(q2(:,i)) * inv(q1);
                            qp(:,i) = q.v(:);
                        end
                    elseif length(q2) == 1
                        for i=1:length(q1)
                            q = q1(i) * Quaternion.pure(q2(:)) * inv(q1(i));
                            qp(:,i) = q.v(:);
                        end
                    else
                        error('RTB:UnitQuaternion:badarg', 'quaternion-double product: vectors lengths incorrect');
                    end
                else
                    error('RTB:UnitQuaternion:badarg', 'quaternion-double product: must be a 3-vector');
                end
            else
                error('RTB:UnitQuaternion:badarg', 'quaternion product: incorrect right hand operand');
            end
        end
        
        function qp = times(q1, q2)
            %Quaternion.times Multiply a quaternion object and unitize
            %
            % Q1.*Q2   is a guaranteed unit quaternion formed by the Hamilton product of two quaternions.
            %
            % Notes::
            % - Overloaded operator '.*'
            % - For case Q1.*Q2 both can be an N-vector, result is elementwise
            %   multiplication.
            % - For case Q1.*Q2 if Q1 scalar and Q2 a vector, scalar multiplies each
            %   element.
            % - For case Q1.*Q2 if Q2 scalar and Q1 a vector, each element multiplies
            %   scalar.        % - If the two multiplicands are unit-quaternions, the product will be a
            %   unit quaternion since it is explicitly enforced.
            %
            % See also Quaternion.mtimes.
            if isa(q2, 'UnitQuaternion')
                qp = unit( q1*q2 );
            else
                error('RTB:UnitQuaternion:badarg', 'quaternion product .*: incorrect operands');
            end
        end
        
        
        function qq = mrdivide(q1, q2)
            %UnitQuaternion.mrdivide Quaternion quotient.
            %
            % Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
            %
            % Notes::
            % - Overloaded operator '/'
            % - For case Q1/Q2 both can be an N-vector, result is elementwise
            %   division.
            % - For case Q1/Q2 if Q1 scalar and Q2 a vector, scalar is divided by each
            %   element.
            % - For case Q1/Q2 if Q2 scalar and Q1 a vector, each element divided by
            %   scalar.
            % - If the dividend and divisor are unit-quaternions, the quotient will be a
            %   unit quaternion.
            %
            % See also Quaternion.mtimes, Quaternion.mpower, Quaternion.plus, Quaternion.minus.
            
            if isa(q2, 'UnitQuaternion')
                % qq = q1 / q2
                %    = q1 * qinv(q2)
                
                qq = q1 * inv(q2);
            else
                error('RTB:UnitQuaternion:badarg', 'quaternion divide /: incorrect RH operands');
            end
        end

        function qp = rdivide(q1, q2)
            %UnitQuaternion.rdivide Quaternion quotient and unitize
            %
            % Q1./Q2   is a quaternion formed by the Hamilton product of two quaternions.
            %
            % Notes::
            % - Overloaded operator '.*'
            % - For case Q1./Q2 both can be an N-vector, result is elementwise
            %   division.
            % - For case Q1./Q2 if Q1 scalar and Q2 a vector, scalar is divided by each
            %   element.
            % - For case Q1./Q2 if Q2 scalar and Q1 a vector, each element divided by
            %   scalar.
            % - The res will be a
            %   unit quaternion since it is explicitly enforced.
            %
            % See also Quaternion.mtimes.
            if isa(q2, 'UnitQuaternion')
                qp = unit( q1/q2 );
            else
                error('RTB:UnitQuaternion:badarg', 'quaternion product .*: incorrect operands');
            end
        end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% TYPE CONVERSION METHODS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

        function s = char(q)
            %UnitQuaternion.char Convert to string
            %
            % S = Q.char() is a compact string representation of the quaternion's value
            % as a 4-tuple.  If Q is a vector then S has one line per element.
            %
            % See also Quaternion.char.
            
            if length(q) > 1
                s = '';
                for qq = q;
                    s = char(s, char(qq));
                end
                return
            end
            s = [num2str(q.s), ' < ' ...
                num2str(q.v(1)) ', ' num2str(q.v(2)) ', '   num2str(q.v(3)) ' >'];
        end  
        
        function r = R(q)
            %UnitQuaternion.R Convert to orthonormal rotation matrix
            %
            % R = Q.R() is the equivalent SO(3) orthonormal rotation matrix (3x3).  If
            % Q represents a sequence (Nx1) then R is 3x3xN.
            %
            % See also UnitQuaternion.T, UnitQuaternion.SO3.
            
            r = zeros(3,3,numel(q));
            for i=1:numel(q)
                r(:,:,i) = q(i).torot();
            end
        end
        
        function t = T(q)
            %UnitQuaternion.T Convert to homogeneous transformation matrix
            %
            % T = Q.T() is the equivalent SE(3) homogeneous transformation matrix
            % (4x4).    If Q represents a sequence (Nx1) then T is 4x4xN.
            %
            % Notes:
            % - Has a zero translational component.
            %
            % See also UnitQuaternion.R, UnitQuaternion.SE3.
            
            t = zeros(4,4,numel(q));
            for i=1:numel(q)
                t(1:3,1:3,i) = q(i).torot();
                t(4,4,i) = 1;
            end
        end
        
        function obj = SO3(q)
            %UnitQuaternion.SO3 Convert to SO3 object
            %
            % X = Q.SO3() is the equivalent SO3 object.
            %
            % Notes::
            % - If Q is a vector then an equivalent vector of SO3 objects is created.
            % See also UnitQuaternion.SE3, SO3.
            
            obj = repmat(SO3, 1, length(q));
            for i=1:numel(q)
                obj(i) = SO3( q(i).R );
            end
        end

        function obj = SE3(q)
            %UnitQuaternion.SE3 Convert to SE3 object
            %
            % X = Q.SE3() is the equivalent SE3 object.
            %
            % Notes::
            % - The translational part of the SE3 object is zero
            % - If Q is a vector then an equivalent vector of SE3 objects is created.
            % See also UnitQuaternion.SE3, SE3.
            
            obj = repmat(SE3, 1, length(q));
            for i=1:numel(q)
                obj(i) = SE3( q(i).T );
            end
        end
    end % methods
    
    methods (Access=private)
        function R = torot(q)
            %TOROT   Convert unit-quaternion to homogeneous transform
            %
            %   T = q2tr(Q)
            %
            %   Return the rotational homogeneous transform corresponding to the unit
            %   quaternion Q.
            %
            %   See also: TR2Q
            
            q = double(q);
            s = q(1);
            x = q(2);
            y = q(3);
            z = q(4);
            
            R = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
                2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
                2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];
            
        end
    end % private methods
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% STATIC FACTORY METHODS, ALTERNATIVE CONSTRUCTORS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)        

        function uq = Rx(varargin)
            %UnitQuaternion.Rx Unit-quaternion constructor for rotation about x-axis.
            %
            % Q = UnitQuaternion.Rx(ANGLE) is a unit-quaternion representing rotation of ANGLE about the x-axis.
            %
            % Q = UnitQuaternion.Rx(ANGLE, 'deg') as above but THETA is in degrees.
            %
            % See also UnitQuaternion.Ry, UnitQuaternion.Rz.
            uq = UnitQuaternion( rotx(varargin{:}));
        end
        
        function uq = Ry(varargin)
            %UnitQuaternion.Ry Unit-quaternion constructor for rotation about y-axis.
            %
            % Q = UnitQuaternion.Ry(ANGLE) is a unit-quaternion representing rotation of ANGLE about the y-axis.
            %
            % Q = UnitQuaternion.Ry(ANGLE, 'deg') as above but THETA is in degrees.
            %
            % See also UnitQuaternion.Rx, UnitQuaternion.Rz.
            uq = UnitQuaternion( roty(varargin{:}));
        end
        
        function uq = Rz(varargin)
            %UnitQuaternion.Rz Unit-quaternion constructor for rotation about z-axis.
            %
            % Q = UnitQuaternion.Rz(ANGLE) is a unit-quaternion representing rotation of ANGLE about the z-axis.
            %
            % Q = UnitQuaternion.Rz(ANGLE, 'deg') as above but THETA is in degrees.
            %
            % See also UnitQuaternion.Rx, UnitQuaternion.Ry.
            uq = UnitQuaternion( rotx(varargin{:}));
        end
        
        function uq = omega(w)
            %UnitQuaternion.omega Unit-quaternion constructor for rotation in angle-vector form.
            %
            % Q = UnitQuaternion.omega(W) is a unit-quaternion representing rotation of |W| about the vector W (3x1).
            %
            % See also UnitQuaternion.angvec.            
            if ~isvec(w)
                error('RTB:UnitQuaternion:bad arg', 'must be a 3-vector');
            end
            theta = norm(w);
            s = cos(theta/2);
            v = sin(theta/2)*unit(w(:)');
            uq = UnitQuaternion(s, v);
        end
        
        function uq = angvec(theta, v)
            %UnitQuaternion.angvec Unit-quaternion constructor for rotation in angle-vector form.
            %
            % Q = UnitQuaternion.omega(TH, V) is a unit-quaternion representing rotation of TH about the vector V (3x1).
            %
            % See also UnitQuaternion.omega.              
            if ~isvec(v)
                error('RTB:UnitQuaternion:bad arg', 'must be a 3-vector');
            end
            uq = UnitQuaternion();
            uq.s = cos(theta/2);
            uq.v = sin(theta/2)*unit(v(:)');
        end
        
        function uq = rpy(varargin)
            %UnitQuaternion.rpy Unit-quaternion constructor for rotation in roll-pitch-yaw angle form.
            %
            % Q = UnitQuaternion.rpy(ROLL, PITCH, YAW, OPTIONS) is a unit-quaternion
            % representing rotation equivalent to the specified roll, pitch, yaw angles
            % angles. These correspond to rotations about the Z, Y, X axes
            % respectively.
            %
            % Q = UnitQuaternion.rpy(RPY, OPTIONS) as above but the angles angles are
            % taken from consecutive columns of the passed matrix RPY = [ROLL, PITCH, YAW].
            % 
            % Options::
            % 'deg'   Compute angles in degrees (radians default)
            % 'xyz'   Return solution for sequential rotations about X, Y, Z axes.
            %
            % Notes::
            % - Is vectorised, see rpy2r for details.
            %
            % See also UnitQuaternion.eul, rpy2r.
            [s,v] = UnitQuaternion.tr2q( rpy2r(varargin{:}) );
            
            uq = UnitQuaternion();
            uq.s = s;
            uq.v = v;
            
        end
        
        function uq = eul(varargin)
            %UnitQuaternion.eul Unit-quaternion constructor for rotation in Euler angle form.
            %
            % Q = UnitQuaternion.eul(PHI, THETA, PSI, OPTIONS) is a unit-quaternion
            % representing rotation equivalent to the specified roll, pitch, yaw angles
            % angles. These correspond to rotations about the Z, Y, Z axes
            % respectively.
            %
            % Q = UnitQuaternion.eul(EUL, OPTIONS) as above but the angles angles are
            % taken from consecutive columns of the passed matrix RPY = [ROLL, PITCH, YAW].
            % 
            % Options::
            % 'deg'   Compute angles in degrees (radians default)
            %
            % Notes::
            % - Is vectorised, see eul2r for details.
            %
            % See also UnitQuaternion.rpy, eul2r.
            [s,v] = UnitQuaternion.tr2q( eul2r(varargin{:}) );
            
            uq = UnitQuaternion();
            uq.s = s;
            uq.v = v;
        end
        
        
        function [s,v] = tr2q(R)
            %TR2Q   Convert homogeneous transform to a unit-quaternion
            %
            %   Q = tr2q(T)
            %
            %   Return a unit-quaternion corresponding to the rotational part of the
            %   homogeneous transform T.
            %
            
            s = sqrt(trace(R)+1)/2.0;
            kx = R(3,2) - R(2,3);   % Oz - Ay
            ky = R(1,3) - R(3,1);   % Ax - Nz
            kz = R(2,1) - R(1,2);   % Ny - Ox
            
            if (R(1,1) >= R(2,2)) && (R(1,1) >= R(3,3))
                kx1 = R(1,1) - R(2,2) - R(3,3) + 1; % Nx - Oy - Az + 1
                ky1 = R(2,1) + R(1,2);          % Ny + Ox
                kz1 = R(3,1) + R(1,3);          % Nz + Ax
                add = (kx >= 0);
            elseif (R(2,2) >= R(3,3))
                kx1 = R(2,1) + R(1,2);          % Ny + Ox
                ky1 = R(2,2) - R(1,1) - R(3,3) + 1; % Oy - Nx - Az + 1
                kz1 = R(3,2) + R(2,3);          % Oz + Ay
                add = (ky >= 0);
            else
                kx1 = R(3,1) + R(1,3);          % Nz + Ax
                ky1 = R(3,2) + R(2,3);          % Oz + Ay
                kz1 = R(3,3) - R(1,1) - R(2,2) + 1; % Az - Nx - Oy + 1
                add = (kz >= 0);
            end
            
            if add
                kx = kx + kx1;
                ky = ky + ky1;
                kz = kz + kz1;
            else
                kx = kx - kx1;
                ky = ky - ky1;
                kz = kz - kz1;
            end
            nm = norm([kx ky kz]);
            if nm == 0
                s = 1;
                v = [0 0 0];
            else
                v = [kx ky kz] * sqrt(1 - s^2) / nm;
            end
        end
    end % static methods
    
end % classdef


