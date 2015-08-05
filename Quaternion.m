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

classdef Quaternion

    properties (SetAccess = protected)
        s       % scalar part
        v       % vector part
    end

    methods

        function q = Quaternion(s, v)
        %Quaternion.Quaternion Constructor for quaternion objects
        %
        % Q = Quaternion is a zero quaternion
        %
        % Q = Quaternion([S V1 V2 V3]) is a quaternion formed by specifying directly its 4 elements
        %
        % Q = Quaternion(S, V) is a quaternion formed from the scalar S and vector
        % part V (1x3)
        %
        % Notes::
        % - The constructor does not handle the vector case.

        if nargin == 0
            q.v = [0,0,0];
            q.s = 0;
        elseif isa(s, 'Quaternion')
            q.s = s.s;
            q.v = s.v;
        elseif nargin == 2 && isscalar(s) && isvec(v,3)
            q.s = s;
            q.v = v(:)';
        elseif nargin == 1 && isvec(s,4)
            s = s(:)';
            q.s = s(1);
            q.v = s(2:4);
        else
            error ('RTB:Quaternion:badarg', 'bad argument to quaternion constructor');
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
            s = [num2str(q.s), ' << ' ...
                num2str(q.v(1)) ', ' num2str(q.v(2)) ', '   num2str(q.v(3)) ' >>'];
        end


        function display(q)
        %Quaternion.display Display quaternion 
        %
        % Q.display() displays a compact string representation of the quaternion's value
        % as a 4-tuple.  If Q is a vector then S has one line per element.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result
        %   of an expression is a Quaternion object and the command has no trailing
        %   semicolon.
        %
        % See also Quaternion.char.


            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(q))
            if loose
                disp(' ');
            end
        end


        function v = double(q)
        %Quaternion.double Convert a quaternion to a 4-element vector
        %
        % V = Q.double() is a 4-vector comprising the quaternion.
        %
        % Notes::
        % - Supports quaternion vector, result is 6xN matrix.
        %
        % elements [s vx vy vz].

            for i=1:length(q)
                v(i,:) = [q(i).s q(i).v];
            end
        end
        


        function qu = unit(q)
        %Quaternion.unit Unitize a quaternion
        %
        % QU = Q.unit() is a unit-quaternion representing the same orientation as Q.
        %
        % Notes::
        % - Supports quaternion vector.
        %
        % See also Quaternion.norm.

            for i=1:length(q)
                qu(i) = q(i) / norm(q(i));
            end
        end

        function n = norm(q)
        %Quaternion.norm Quaternion magnitude
        %
        % QN = Q.norm(Q) is the scalar norm or magnitude of the quaternion Q.  
        %
        % Notes::
        % - This is the Euclidean norm of the quaternion written as a 4-vector.
        % - A unit-quaternion has a norm of one.
        %
        % See also Quaternion.inner, Quaternion.unit.

            n = colnorm(double(q)')';
        end

        function n = inner(q1, q2)
        %Quaternion.inner Quaternion inner product
        %
        % V = Q1.inner(Q2) is the inner (dot) product of two vectors (1x4),
        % comprising the elements of Q1 and Q2 respectively.
        %
        % Notes::
        % - Q1.inner(Q1) is the same as Q1.norm().
        %
        % See also Quaternion.norm.

            n = double(q1)*double(q2)';
        end

        


        function e = eq(q1, q2)
        %EQ Test quaternion equality
        %
        % Q1==Q2 is true if the quaternions Q1 and Q2 are equal.
        %
        % Notes::
        % - Overloaded operator '=='.
        % - Note that for unit Quaternions Q and -Q are the equivalent
        %   rotation, so non-equality does not mean rotations are not
        %   equivalent.
        % - If Q1 is a vector of quaternions, each element is compared to 
        %   Q2 and the result is a logical array of the same length as Q1.
        % - If Q2 is a vector of quaternions, each element is compared to 
        %   Q1 and the result is a logical array of the same length as Q2.
        % - If Q1 and Q2 are vectors of the same length, then the result 
        %   is a logical array of the same length.
        %
        % See also Quaternion.ne.
            if (numel(q1) == 1) && (numel(q2) == 1)
                e = all( eq(q1.double, q2.double) );
            elseif (numel(q1) >  1) && (numel(q2) == 1)
                e = zeros(1, numel(q1));
                for i=1:numel(q1)
                    e(i) = q1(i) == q2;
                end
            elseif (numel(q1) == 1) && (numel(q2) > 1)
                e = zeros(1, numel(q2));
                for i=1:numel(q2)
                    e(i) = q2(i) == q1;
                end
            elseif numel(q1) == numel(q2)
                e = zeros(1, numel(q1));
                for i=1:numel(q1)
                    e(i) = q1(i) == q2(i);
                end
            else
                error('RTB:quaternion:badargs');
            end
        end

        function e = ne(q1, q2)
        %NE Test quaternion inequality
        %
        % Q1~=Q2 is true if the quaternions Q1 and Q2 are not equal.
        %
        % Notes::
        % - Overloaded operator '~='
        % - Note that for unit Quaternions Q and -Q are the equivalent
        %   rotation, so non-equality does not mean rotations are not
        %   equivalent.
        % - If Q1 is a vector of quaternions, each element is compared to 
        %   Q2 and the result is a logical array of the same length as Q1.
        % - If Q2 is a vector of quaternions, each element is compared to 
        %   Q1 and the result is a logical array of the same length as Q2.
        % - If Q1 and Q2 are vectors of the same length, then the result 
        %   is a logical array of the same length.
        %
        % See also Quaternion.eq.
            if (numel(q1) == 1) && (numel(q2) == 1)
                e = all( ne(q1.double, q2.double) );
            elseif (numel(q1) >  1) && (numel(q2) == 1)
                e = zeros(1, numel(q1));
                for i=1:numel(q1)
                    e(i) = q1(i) ~= q2;
                end
            elseif (numel(q1) == 1) && (numel(q2) > 1)
                e = zeros(1, numel(q2));
                for i=1:numel(q2)
                    e(i) = q2(i) ~= q1;
                end
            elseif numel(q1) == numel(q2)
                e = zeros(1, numel(q1));
                for i=1:numel(q1)
                    e(i) = q1(i) ~= q2(i);
                end
            else
                error('RTB:quaternion:badargs');
            end
        end

        function qp = plus(q1, q2)
        %PLUS Add quaternions
        %
        % Q1+Q2 is the element-wise sum of quaternion elements.
        %
        % Notes::
        % - Overloaded operator '+'
        % - The result is not guaranteed to be a unit-quaternion.
        %
        % See also Quaternion.minus, Quaternion.mtimes.
        
            if isa(q1, 'Quaternion') && isa(q2, 'Quaternion')
                qp = Quaternion(double(q1) + double(q2));
            elseif isa(q1, 'Quaternion') && isvec(q2, 4)
                    qp = Quaternion(q1);
                    q2 = q2(:)';
                    qp.s = qp.s + q2(1);
                    qp.v = qp.v + q2(2:4);
            end
        end


        function qp = minus(q1, q2)
        %Quaternion.minus Subtract quaternions
        %
        % Q1-Q2 is the element-wise difference of quaternion elements.
        %
        % Notes::
        % - Overloaded operator '-'
        % - The result is not guaranteed to be a unit-quaternion.
        %
        % See also Quaternion.plus, Quaternion.mtimes.

            if isa(q1, 'Quaternion') && isa(q2, 'Quaternion')

                qp = Quaternion(double(q1) - double(q2));
            end
        end
        


        function qp = mtimes(q1, q2)
            %Quaternion.mtimes Multiply a quaternion object
            %
            % Q1*Q2   is a quaternion formed by the Hamilton product of two quaternions.
            % Q*V     is a vector formed by rotating the vector V by the quaternion Q.
            % Q*S     is the element-wise multiplication of quaternion elements by the scalar S.
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
            
            if isa(q2, 'Quaternion')
                %QQMUL  Multiply unit-quaternion by unit-quaternion
                %
                %   QQ = qqmul(Q1, Q2)
                %
                %   Return a product of unit-quaternions.
                %
                %   See also: TR2Q
                
                if length(q1) == length(q2)
                    for i=1:length(q1)
                        % decompose into scalar and vector components
                        s1 = q1(i).s;  v1 = q1(i).v;
                        s2 = q2(i).s;  v2 = q2(i).v;
                        
                        % form the product
                        qp(i) = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);
                    end
                elseif isscalar(q1)
                    s1 = q1.s;  v1 = q1.v;
                    
                    for i=1:length(q2)
                        % decompose into scalar and vector components
                        s2 = q2(i).s;  v2 = q2(i).v;
                        
                        % form the product
                        qp(i) = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);
                    end
                elseif isscalar(q2)
                    s2 = q2.s;  v2 = q2.v;
                    
                    for i=1:length(q1)
                        % decompose into scalar and vector components
                        s1 = q1(i).s;  v1 = q1(i).v;
                        
                        % form the product
                        qp(i) = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);
                    end
                else
                    error('RTB:quaternion:badarg', '* operand length mismatch');
                end
                
            elseif isa(q1, 'Quaternion') && isa(q2, 'double')
                
                %QVMUL  Multiply vector by unit-quaternion
                %
                %   VT = qvmul(Q, V)
                %
                %   Rotate the vector V by the unit-quaternion Q.
                %
                %   See also: QQMUL, QINV
                
               if length(q2) == 1
                    qp = Quaternion( double(q1)*q2);
                else
                    error('RTB:Quaternion:badarg', 'quaternion-double product: must be a scalar');
                end
                
            elseif isa(q2, 'Quaternion') && isa(q1, 'double')
                    if length(q1) == 1
                    qp = Quaternion( double(q2)*q1);
                else
                    error('RTB:Quaternion:badarg', 'quaternion-double product: must be a scalar');
                end
            else
                error('RTB:Quaternion:badarg', 'quaternion product: incorrect right hand operand');
            end
        end
        
        function c = conj(q)
                    %Quaternion.inv Invert a unit-quaternion
        %
        % QI = Q.inv() is a quaternion object representing the inverse of Q.
        %
        % Notes::
        % - Supports quaternion vector.
            c = Quaternion(q.s, -q.v);
        end

                function qi = inv(q)
        %Quaternion.inv Invert a quaternion
        %
        % QI = Q.inv() is a quaternion object representing the inverse of Q.
        %
        % Notes::
        % - Supports quaternion vector.

            for i=1:length(q)
                n2 = sum( q(i).double.^2 );
                qi(i) = Quaternion([q(i).s -q(i).v]/ n2);
            end
                end
        
        function qp = mpower(q, p)
        %Quaternion.mpower Raise quaternion to integer power
        %
        % Q^N is the quaternion Q raised to the integer power N.
        %
        % Notes::
        % - Overloaded operator '^'
        % - Computed by repeated multiplication.
        % - If the argument is a unit-quaternion, the result will be a
        %   unit quaternion.
        %
        % See also Quaternion.mrdivide, Quaternion.mpower, Quaternion.plus, Quaternion.minus.

            % check that exponent is an integer
            if (p - floor(p)) ~= 0
                error('quaternion exponent must be integer');
            end

            qp = q;

            % multiply by itself so many times
            for i = 2:abs(p)
                qp = qp * q;
            end

            % if exponent was negative, invert it
            if p<0
                qp = inv(qp);
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

            if isa(q2, 'Quaternion')
                % qq = q1 / q2
                %    = q1 * qinv(q2)

                qq = q1 * inv(q2);
            elseif isa(q2, 'double')
                qq = Quaternion( double(q1) / q2 );
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
        
        function hout = plot(Q, varargin)
        %Quaternion.plot Plot a quaternion object 
        %
        % Q.plot(options) plots the quaternion as an oriented coordinate frame.
        %
        % h = Q.plot(options) as above but returns a handle which can be used for animation.
        %
        % Options::
        % Options are passed to trplot and include:
        %
        % 'color',C          The color to draw the axes, MATLAB colorspec C
        % 'frame',F          The frame is named {F} and the subscript on the axis labels is F.
        % 'view',V           Set plot view parameters V=[az el] angles, or 'auto' 
        %                    for view toward origin of coordinate frame
        % 'handle',h         Update the specified handle
        %
        % See also trplot.

            %axis([-1 1 -1 1 -1 1])

            h = trplot( Q.R, varargin{:});
            if nargout > 0
                hout = h;
            end
        end

        function animate(Q, varargin)
        %Quaternion.animate Animate a quaternion object
        %
        % Q.animate(options) animates a quaternion array Q as a 3D coordinate frame.
        %
        % Q.animate(QF, options) animates a 3D coordinate frame moving from
        % orientation Q to orientation QF.
        %
        % Options::
        % Options are passed to tranimate and include:
        %
        %  'fps', fps    Number of frames per second to display (default 10)
        %  'nsteps', n   The number of steps along the path (default 50)
        %  'axis',A      Axis bounds [xmin, xmax, ymin, ymax, zmin, zmax]
        %  'movie',M     Save frames as files in the folder M
        %  'cleanup'     Remove the frame at end of animation
        %  'noxyz'       Don't label the axes
        %  'rgb'         Color the axes in the order x=red, y=green, z=blue
        %  'retain'      Retain frames, don't animate
        %  Additional options are passed through to TRPLOT.
        %
        % See also tranimate, trplot.
        
            if nargin > 1 && isa(varargin{1}, 'Quaternion')
                QF = varargin{1};
                tranimate(Q.R, QF.R, varargin{2:end});
            else
                tranimate(Q.R, varargin{:});
            end
        end
           
        function qd = dot(q, omega)
        %Quaternion.dot Quaternion derivative
        %
        % QD = Q.dot(omega) is the rate of change of a frame with attitude Q and
        % angular velocity OMEGA (1x3) expressed as a quaternion.
            E = q.s*eye(3,3) - skew(q.v);
            omega = omega(:);
            qd = Quaternion([-0.5*q.v*omega; 0.5*E*omega]);
        end
    end % methods
    
    methods(Static)
        function q = pure(v)
            
            if ~isvec(v)
                error('RTB:Quaternion:bad arg', 'must be a 3-vector');
            end
            q = Quaternion(0, v(:));
        end
    end
end
