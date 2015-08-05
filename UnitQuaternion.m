%QUATERNION Quaternion class
% 
% A quaternion is a compact method of representing a 3D rotation that has
% computational advantages including speed and numerical robustness.
% A quaternion has 2 parts, a scalar s, and a vector v and is typically 
% written: q = s <vx, vy, vz>.  
%
% A unit-quaternion is one for which s^2+vx^2+vy^2+vz^2 = 1.  It can be 
% considered as a rotation by an angle theta about a unit-vector V in space where 
%
%         q = cos (theta/2) < v sin(theta/2)> 
%
% Q = Quaternion(X) is a unit-quaternion equivalent to X which can be any
% of:
%   - orthonormal rotation matrix.
%   - homogeneous transformation matrix (rotation part only).
%   - rotation angle and vector
%
% Methods::
%  inv       inverse of quaterion
%  norm      norm of quaternion
%  unit      unitized quaternion
%  plot      plot a coordinate frame representing orientation of quaternion
%  animate   animates a coordinate frame representing changing orientation
%            of quaternion sequence
%  angvec    convert to angle vector form
%  interp    interpolation (slerp) between q and q2, 0<=s<=1
%  scale     interpolation (slerp) between identity and q, 0<=s<=1
%  dot       derivative of quaternion with angular velocity w
%  R         equivalent 3x3 rotation matrix
%  T         equivalent 4x4 homogeneous transform matrix
%  double    quaternion elements as 4-vector
%  inner     inner product of two quaternions
%
% Overloaded operators::
%  q1==q2    test for quaternion equality
%  q1~=q2    test for quaternion inequality
%  q+q2      elementwise sum of quaternions
%  q-q2      elementwise difference of quaternions
%  q*q2      quaternion (Hamilton) product
%  q.*q2     quaternion (Hamilton) product followed by unitization
%  q*v       rotate vector by quaternion, v is 3x1
%  s*q       elementwise multiplication of quaternion by scalar
%  q/q2      q*q2.inv
%  q^n       q to power n (integer only)
%
% Properties (read only)::
%  s         real part
%  v         vector part
%
% Notes::
% - Quaternion objects can be used in vectors and arrays.
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
% See also trinterp, trplot.

% TODO
% properties s, v for the vector case


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
        %Quaternion.Quaternion Constructor for quaternion objects
        % 
        % Construct a quaternion from various other orientation representations.
        %
        % Q = Quaternion() is the identitity unit-quaternion 1<0,0,0> representing a null rotation.
        %
        % Q = Quaternion('quaternion', Q1) is a copy of the quaternion Q1
        %
        % Q = Quaternion('euler', EUL) create a unit-quaternion equivalent to the Euler
        % angle vector EUL (1x3).
        %
        % Q = Quaternion('euler', E1, E2, E3) as above but Euler angles are given by
        % three scalars.
        %
        % Q = Quaternion('RPY', RPY) create a unit-quaternion equivalent to the
        % roll-pitch-yaw angle vector RPY (1x3).
        %
        % Q = Quaternion('RPY', R, P, Y) as above but RPY angles are given by three
        % scalars.
        %
        % Q = Quaternion('angvec', TH, V) is a unit-quaternion corresponding to rotation of TH about the 
        % vector V.
        %
        % Q = Quaternion('omega', W) is a unit-quaternion corresponding to rotation of |W| about the 
        % vector W.
        %
        % Q = Quaternion('pure', V) is a pure quaternion with the specified vector part: 0<V>
        %
        % Q = Quaternion('component', [S V1 V2 V3]) is a quaternion formed by
        % specifying directly its 4 components
        %
        %
        % Old style arguments::
        %
        % Q = Quaternion([S V1 V2 V3]) is a quaternion formed by specifying directly its 4 elements
        %
        % Q = Quaternion(S) is a quaternion formed from the scalar S and zero vector part: S<0,0,0>
        %
        % Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
        %
        % Q = Quaternion(TH, V) is a unit-quaternion corresponding to rotation of TH about the 
        % vector V.
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
        % - The constructor does not handle the vector case.

        if nargin == 0
            uq.v = [0,0,0];
            uq.s = 1;
        elseif isa(s, 'Quaternion')
            uq = UnitQuaternion();
            if ~isa(s, 'UnitQuaternion');
                s = s.unit();
            end
            uq.s = s.s;
            uq.v = s.v;
        elseif isrot(s) || ishomog(s)
            %   Q = Quaternion(R)       from a 3x3 or 4x4 matrix
            for i=1:size(s,3)
                uq(i) = Quaternion( Quaternion.tr2q(s(:,:,i)) );
            end
        elseif nargin ==2 && isscalar(s) && isvec(v,3)
            % ensure its a unit quaternion
            n = norm([s; v(:)]);
            uq.s = s/n;
            uq.v = v/n;
            else
            
            error ('RTB:UnitQuaternion:badarg', 'bad argument to quaternion constructor');
        end
        
    end
        
        function [theta_,n_] = toangvec(q, varargin)
        %Quaternion.angvec Convert to angle-vector form
        %
        % Q.angvec(OPTIONS) prints a compact one representation of the rotational angle and rotation vector 
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
        
            opt.deg = false;
            opt = tb_optparse(opt, varargin);
            
            if norm(q.v) < 10*eps
                % identity quaternion, null rotation
                theta = 0;
                n = [0 0 0];
            else
                % finite rotation
                n = unit(q.v);
                theta = 2*atan2( norm(q.v), q.s);
            end
            
            if opt.deg
                theta = theta * 180/pi;
                units = 'deg';
            else
                units = 'rad';
            end
            
            if nargout == 0
                % if no output arguments display the angle and vector
                fprintf('Rotation: %f %s x [%f %f %f]\n', theta, units, n);
            elseif nargout == 1
                theta_ = theta;
            elseif nargout == 2
                theta_ = theta;
                n_ = n;
            end
        end


        function s = char(q)
        %Quaternion.char Convert to string
        %
        % S = Q.char() is a compact string representation of the quaternion's value
        % as a 4-tuple.  If Q is a vector then S has one line per element.

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
        
        function qp = mtimes(q1, q2)
            %Quaternion.mtimes Multiply a quaternion object
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
                qp = UnitQuaternion(mtimes@Quaternion(q1, q2));
                
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
                
                if length(q2) == 3
                    qp = q1 * Quaternion.pure(q2) * inv(q1);
                    qp = qp.v(:);
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
                
        function qp = mpower(q1, q2)
            qp = UnitQuaternion(mpower@Quaternion(q1, q2));
        end
        
                function qp = rdivide(q1, q2)
        %Quaternion.rdivide Quaternion quotient and unitize
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
        % - If the two multiplicands are unit-quaternions, the product will be a
        %   unit quaternion since it is explicitly enforced.
        %
        % See also Quaternion.mtimes.
            if isa(q2, 'UnitQuaternion')
                qp = unit( q1/q2 );
            else
                error('RTB:UnitQuaternion:badarg', 'quaternion product .*: incorrect operands');
            end
                end
                function qq = mrdivide(q1, q2)
        %Quaternion.mrdivide Quaternion quotient.
        %
        % Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
        % Q/S     is the element-wise division of quaternion elements by the scalar S.
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
        function qi = inv(q)
        %Quaternion.inv Invert a unit-quaternion
        %
        % QI = Q.inv() is a quaternion object representing the inverse of Q.
        %
        % Notes::
        % - Supports quaternion vector.

            for i=1:length(q)
                qi(i) = UnitQuaternion(q(i).s, -q(i).v);
            end
        end

        function q = interp(Q1, Q2, r, varargin)
        %Quaternion.interp Interpolate quaternions
        %
        % QI = Q1.interp(Q2, S, OPTIONS) is a unit-quaternion that interpolates a rotation 
        % between Q1 for S=0 and Q2 for S=1.
        %
        % If S is a vector QI is a vector of quaternions, each element
        % corresponding to sequential elements of S.
        %
        % Options::
        % 'shortest'   Take the shortest path along the great circle
        %
        % Notes::
        % - This is a spherical linear interpolation (slerp) that can be interpretted 
        %   as interpolation along a great circle arc on a sphere.
        % - The value of S is clipped to the interval 0 to 1.
        %
        % References::
        % - Animating rotation with quaternion curves,
        %   K. Shoemake,
        %   in Proceedings of ACM SIGGRAPH, (San Fran cisco), pp. 245-254, 1985.
        %
        % See also Quaternion.scale, ctraj.

            q1 = double(Q1);
            q2 = double(Q2);
            
            opt.shortest = false;
            
            opt = tb_optparse(opt, varargin);
            

            cosTheta = q1*q2';
            
            if opt.shortest
                % take shortest path along the great circle, patch by Gauthier Gras
                if cosTheta < 0
                    q1 = - q1;
                    cosTheta = - cosTheta;
                end;
            end
            
            theta = acos(cosTheta);
            count = 1;

            % clip values of r
            r(r<0) = 0;
            r(r>1) = 1;
           
            
            q(length(r)) = Quaternion();  % preallocate space for Quaternion vector
            
            for i=1:length(r)
                if theta == 0
                    q(i) = Q1;
                else
                    q(i) = Quaternion( (sin((1-r(i))*theta) * q1 + sin(r(i)*theta) * q2) / sin(theta) );
                end
            end
        end
        function q = scale(Q, r)
        %Quaternion.scale Interpolate rotations expressed by quaternion objects
        %
        % QI = Q.scale(S) is a unit-quaternion that interpolates between a null
        % rotation (identity quaternion) for S=0 to Q for S=1.  This is a spherical
        % linear interpolation (slerp) that can be interpretted as interpolation
        % along a great circle arc on a sphere.
        %
        % If S is a vector QI is a vector of quaternions, each element
        % corresponding to sequential elements of S.
        %
        % Notes::
        % - This is a spherical linear interpolation (slerp) that can be interpretted 
        %   as interpolation along a great circle arc on a sphere.
        %
        % See also Quaternion.interp, ctraj.


            q2 = double(Q);

            if any(r<0) || (r>1)
                error('r out of range');
            end
            q1 = [1 0 0 0];         % identity quaternion
            theta = acos(q1*q2');

            if length(r) == 1
                if theta == 0
                    q = Q;
                else
                    q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
                end
            else
                count = 1;
                for R=r(:)'
                    if theta == 0
                        qq = Q;
                    else
                        qq = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
                    end
                    q(count) = qq;
                    count = count + 1;
                end
            end
        end        
        

        function th = theta(Q)
        %Quaternion.theta Rotation angle of quaternion.
        %
        % Q1.theta   is a quaternion formed by Hamilton product of Q1 and inv(Q2).
        % Q/S     is the element-wise division of quaternion elements by the scalar S.
        %
        % Notes::
        % - Supports vector of quaternions, result is 1xN vector.
        %
        % See also Quaternion.angvec.

            % get the scalar part and clip it, just in case it's unnormalized
            s = [Q.s];
            s(s<-1) = -1;
            s(s>1) = 1;
            % get the angle
            th = 2*acos(s);
        end
        
            
        function r = R(q)
        %Quaternion.R Convert to orthonormal rotation matrix
        %
        % R = Q.R() is the equivalent SO(3) orthonormal rotation matrix (3x3).  If
        % Q represents a sequence (Nx1) then R is 3x3xN.
        %

            r = zeros(3,3,numel(q));
            for i=1:numel(q)
                r(:,:,i) = t2r( q2tr(q(i)) );
            end
        end

        function t = T(q)
        %Quaternion.T Convert to homogeneous transformation matrix
        %
        % T = Q.T() is the equivalent SE(3) homogeneous transformation matrix
        % (4x4).    If Q represents a sequence (Nx1) then T is 4x4xN.
        %
        % Notes:
        % - Has a zero translational component.
            t = zeros(4,4,numel(q));
            for i=1:numel(q)
                t(:,:,i) = q2tr(q(i));
            end
        end

    end % methods

methods(Static)
    
    function uq = omega(w)
        
        if ~isvec(w)
            error('RTB:UnitQuaternion:bad arg', 'must be a 3-vector');
        end
        theta = norm(w);
        s = cos(theta/2);
        v = sin(theta/2)*unit(w(:)');
        uq = UnitQuaternion(s, v);
    end
    
    function uq = angvec(theta, v)
        
        if ~isvec(v)
            error('RTB:UnitQuaternion:bad arg', 'must be a 3-vector');
        end
        
        s = cos(theta/2);
        v = sin(theta/2)*unit(v(:)');
        uq = UnitQuaternion(s, v); 
    end
    

    
    function uq = rpy(varargin)
        uq = Quaternion.tr2q( rpy2r(varargin{:}) );
    end
    
    function uq = eul(varargin)
        uq = tr2q( eul2r(varargin{:}) );
    end
    

end
end % classdef
    %TR2Q   Convert homogeneous transform to a unit-quaternion
    %
    %   Q = tr2q(T)
    %
    %   Return a unit-quaternion corresponding to the rotational part of the
    %   homogeneous transform T.
    %
    %   See also: Q2TR
    
    function [s,v] = tr2q(t)
        
        if ishomog(t)
            t = t2r(t);
        end
        qs = sqrt(trace(t)+1)/2.0;
        kx = t(3,2) - t(2,3);   % Oz - Ay
        ky = t(1,3) - t(3,1);   % Ax - Nz
        kz = t(2,1) - t(1,2);   % Ny - Ox
        
        if (t(1,1) >= t(2,2)) && (t(1,1) >= t(3,3))
            kx1 = t(1,1) - t(2,2) - t(3,3) + 1; % Nx - Oy - Az + 1
            ky1 = t(2,1) + t(1,2);          % Ny + Ox
            kz1 = t(3,1) + t(1,3);          % Nz + Ax
            add = (kx >= 0);
        elseif (t(2,2) >= t(3,3))
            kx1 = t(2,1) + t(1,2);          % Ny + Ox
            ky1 = t(2,2) - t(1,1) - t(3,3) + 1; % Oy - Nx - Az + 1
            kz1 = t(3,2) + t(2,3);          % Oz + Ay
            add = (ky >= 0);
        else
            kx1 = t(3,1) + t(1,3);          % Nz + Ax
            ky1 = t(3,2) + t(2,3);          % Oz + Ay
            kz1 = t(3,3) - t(1,1) - t(2,2) + 1; % Az - Nx - Oy + 1
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
            q = UnitQuaternion();
        else
            s = sqrt(1 - qs^2) / nm;
            qv = s*[kx ky kz];
            
            q = UnitQuaternion(qs, qv);
            
        end
    end
    
    
    %Q2TR   Convert unit-quaternion to homogeneous transform
    %
    %   T = q2tr(Q)
    %
    %   Return the rotational homogeneous transform corresponding to the unit
    %   quaternion Q.
    %
    %   See also: TR2Q
    
    function t = q2tr(q)
        
        q = double(q);
        s = q(1);
        x = q(2);
        y = q(3);
        z = q(4);
        
        r = [   1-2*(y^2+z^2)   2*(x*y-s*z) 2*(x*z+s*y)
            2*(x*y+s*z) 1-2*(x^2+z^2)   2*(y*z-s*x)
            2*(x*z-s*y) 2*(y*z+s*x) 1-2*(x^2+y^2)   ];
        t = eye(4,4);
        t(1:3,1:3) = r;
        t(4,4) = 1;
    end
    


