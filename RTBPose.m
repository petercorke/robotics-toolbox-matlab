%RTBPose Superclass for SO2, SO3, SE2, SE3
%
% This abstract class provides common methods for the 2D and 3D orientation and pose
% classes: SO2, SE2, SO3 and SE3.
%
% Methods::
%
%  dim         dimension of the underlying matrix
%  isSE        true for SE2 and SE3
%  issym       true if value is symbolic
%  plot        graphically display coordinate frame for pose
%  animate     graphically display coordinate frame for pose
%  print       print the pose in single line format
%  display     print the pose in human readable matrix form
%  char        convert to human readable matrix as a string
%--
%  double      convert to real rotation or homogeneous transformation matrix
%  simplify    apply symbolic simplification to all elements
%  vpa         apply vpa to all elements
%
% Operators::
%  +           elementwise addition, result is a matrix
%  -           elementwise subtraction, result is a matrix
%  *           multiplication within group, also SO3 x vector
%  /           multiplication within group by inverse
%  ==          test equality
%  ~=          test inequality
%
% A number of compatibility methods give the same behaviour as the
% classic RTB functions:
%
%  tr2rt       convert to rotation matrix and translation vector
%  t2r         convert to rotation matrix
%  trprint     print single line representation
%  trprint2    print single line representation
%  trplot      plot coordinate frame
%  trplot2     plot coordinate frame
%  tranimate   aimate coordinate frame
%
% Notes::
% - Multiplication and division with normalization operations are performed
%   in the subclasses.
% - SO3 is polymorphic with UnitQuaternion making it easy to change
%   rotational representations.
% - If the File Exchange function cprintf is available it is used to print
%   the matrix in color: red for rotation and blue for translation.
%
% See also SO2, SO3, SE2, SE3.


% Copyright (C) 1993-2017, by Peter I. Corke
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


classdef (Abstract) RTBPose
    
    properties(Access=protected, Hidden=true)
        data   % this is a 2x2, 3x3 or 4x4 matrix, possibly symbolic
    end
    
    
    methods
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  INFORMATION METHODS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function n = dim(obj)
            %RTBPose.dim  Dimension
            %
            % N = P.dim() is the dimension of the group object, 2 for SO2, 3 for SE2
            % and SO3, and 4 for SE3.
            
            n = size(obj(1).data, 2);
        end
        
        function t = isSE(T)
            %RTBPose.isSE  Test if pose
            %
            % P.isSE() is true if the object is of type SE2 or SE3.
            
            s = class(T);
            t = s(2) == 'E';
        end
        
        function t = issym(obj)
            %RTBPose.issym  Test if pose is symbolic
            %
            % P.issym() is true if the pose has symbolic rather than real values.
            t = isa(obj(1).data, 'sym');
        end
        
%         function e = isequal(obj1, obj2)
%             %ISEQUAL Test quaternion element equality
%             %
%             % ISEQUAL(Q1,Q2) is true if the quaternions Q1 and Q2 are equal.
%             %
%             % Notes::
%             % - Used by test suite verifyEqual in addition to eq().
%             % - Invokes eq().
%             %
%             % See also Quaternion.eq.
%             e = isa(obj2, classname(obj1)) && ...
%                 length(obj1) == length(obj2) &&
%                 alleq(q1, q2);
%         end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  OPERATORS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function v = plus(a, b)
            %RTBPose.plus Add poses
            %
            % P1+P2 is the elementwise summation of the matrix elements of the two
            % poses.  The result is a matrix not the input class type since the result
            % of addition is not in the group.
            
            assert(isa(b, class(a)) && length(a) == length(b), 'RTB:RTBPose:plus:badarg', 'operands don''t conform');
            for i=1:length(a)
                v(:,:,i) = a(i).data + b(i).data;
            end
        end
        
        function v = minus(a, b)
            %RTBPose.minus Subtract poses
            %
            % P1-P2 is the elementwise difference of the matrix elements of the two
            % poses.  The result is a matrix not the input class type since the result
            % of subtraction is not in the group.
            
            assert(isa(b, class(a)) && length(a) == length(b), 'RTB:RTBPose:minus:badarg', 'operands don''t conform');
            for i=1:length(a)
                v(:,:,i) = a(i).data - b(i).data;
            end
        end
        
        function out = simplify(obj)
            %RTBPose.simplify Symbolic simplification
            %
            % P2 = P.simplify() applies symbolic simplification to each element of
            % internal matrix representation of the pose.
            %
            % See also simplify.
            out = obj;
            if isa(obj(1).data, 'sym')
                for k=1:length(obj)
                    % simplify every element of data
                    for i=1:numel(obj.data)
                        out(k).data(i) = simplify( obj(k).data(i) );
                    end
                end
            end
        end
        
        function out = vpa(obj, D)
            %RTBPose.vpa Variable precision arithmetic
            %
            % P2 = P.vpa() numerically evaluates each element of
            % internal matrix representation of the pose.
            %
            % P2 = P.vpa(D) as above but with D decimal digit accuracy.
            %
            % See also simplify.
            out = obj;
            if nargin == 1
                D = digits;
            end
            if isa(obj(1).data, 'sym')
                for k=1:length(obj)
                    % simplify every element of data
                    for i=1:numel(obj.data)
                        out(k).data(i) = vpa( obj(k).data(i), D );
                    end
                end
            end
        end
        
        function e = isequal(obj1, obj2)
            e = eq(obj1, obj2);
        end
        
        function e = eq(obj1, obj2)
            e = false;
            
            if ~isa(obj2, class(obj2)) || ~(length(obj1) == length(obj2))
                return;
            end
            
            assert(length(obj1) == length(obj2), 'RTB:RTBPose:eq', 'arrays must be same size');
            e = zeros(size(obj1), 'logical');
            for i=1:length(obj1)
                e(i) = all(all(abs([obj1(i).data] - [obj2(i).data]) < 10*eps));
            end
            
        end
        
        function e = ne(obj1, obj2)
            e = ~eq(obj1, obj2);
        end
        

        function e = mpower(obj1, n)
            %RTBPose.mpower Exponential of pose
            %
            % P^N is the exponential of P where N is an integer.  It is equivalent of compounding
            % the rigid-body motion of P with itself N-1 times.
            %
            % Notes::
            % - N can be 0 in which case the result is the identity matrix.
            % - N can be negative which is equivalent to the inverse of P^abs(N).
            %
            % See also RTBPose.power, RTBPose.mtimes, RTBPose.times.
            assert(isscalar(n) && isreal(n) && floor(n) == n, 'RTB:Pose', 'exponent must be a real integer');
            e = SE3( double(obj1)^n);
        end
        
        function e = power(obj1, n)
            %RTBPose.power Exponential of pose
            %
            % P.^N is the exponential of P where N is an integer, followed by normalization.  It is equivalent of compounding
            % the rigid-body motion of P with itself N-1 times.
            %
            % Notes::
            % - N can be 0 in which case the result is the identity matrix.
            % - N can be negative which is equivalent to the inverse of P.^abs(N).
            %
            % See also RTBPose.mpower, RTBPose.mtimes, RTBPose.times.
            assert(isscalar(n) && isreal(n) && floor(n) == n, 'RTB:Pose', 'exponent must be a real integer');
            e = SE3( trnorm(double(obj1)^n) );
        end
        
        function e = transpose(obj1, obj2)
            error('RTB:Pose', 'transpose operator not supported by RTBPose subclass object')
        end
        function e = ctranspose(obj1, obj2)
            error('RTB:Pose', 'transpose operator not supported by RTBPose subclass object')
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  COMPOSITION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function out = mtimes(obj, a)
            %RTBPose.mtimes  Compound pose objects
            %
            % R = P*Q is a pose object representing the composition of the two
            % poses described by the objects P and Q, which is  multiplication
            % of their equivalent matrices.
            %
            % If either, or both, of P or Q are vectors, then the result is a vector.
            %
            % If P is a vector (1xN) then R is a vector (1xN) such that R(i) = P(i)*Q.
            %
            % If Q is a vector (1xN) then R is a vector (1xN) such thatR(i) = P*Q(i).
            %
            % If both P and Q are vectors (1xN) then R is a vector (1xN) such that
            % R(i) = P(i)*R(i).
            %
            % W = P*V is a column vector (2x1) which is the transformation of the
            % column vector V (2x1) by the rotation described by the SO2 object P.
            % P can be a vector and/or V can be a matrix, a columnwise set of vectors.
            %
            % If P is a vector (1xN) then W is a matrix (2xN) such that W(:,i) = P(i)*V.
            %
            % If V is a matrix (2xN) V is a matrix (2xN) then W is a matrix (2xN) such
            % that W(:,i) = P*V(:,i).
            %
            % If P is a vector (1xN) and V is a matrix (2xN) then W is a matrix (2xN)
            % such that W(:,i) = P(i)*V(:,i).
            %
            % See also RTBPose.mrdivide.

            if strcmp(class(obj), class(a))
                % obj * obj
                obj1 = obj(1);
                out = repmat(obj1, 1, max(length(obj),length(a)));
                if length(obj) == length(a)
                    % do objvector*objvector and objscalar*objscalar case
                    for i=1:length(obj)
                        out(i) = obj1.new( obj(i).data * a(i).data);
                    end
                elseif length(obj) == 1
                    % objscalar*objvector case
                    for i=1:length(a)
                        out(i) = obj1.new( obj.data * a(i).data);
                    end
                elseif length(a) == 1
                    % objvector*objscalar case
                    for i=1:length(obj)
                        out(i) = obj1.new( obj(i).data * a.data);
                    end
                else
                    error('RTB:RTBPose:badops', 'invalid operand lengths to * operator');
                end
                
            elseif isa(obj, 'RTBPose') && isnumeric(a)
                % obj * vectors (nxN), result is nxN
                assert(isreal(a), 'RTB:RTBPose:*', 'matrix must be real');
                q2 = double(a); % force to double
                
                obj1 = obj(1);
                n = numrows(obj1.data);

                if obj1.isSE
                    % is SE(n) convert to homogeneous form
                    assert(numrows(a) == n-1, 'RTB:RTBPose:badops', 'LHS should be matrix with %d rows', n-1);
                    a = [a; ones(1, numcols(a))];
                else
                    assert(numrows(a) == n, 'RTB:RTBPose:badops', 'LHS should be matrix with %d rows', n);
                end
                
                out = zeros(n, max(length(obj),numcols(a)));  % preallocate space
                
                if length(obj) == numcols(a)
                    % do objvector*vector and objscalar*scalar case
                    for i=1:length(obj)
                        out(:,i) = obj(i).data * a(:,i);
                    end
                elseif length(obj) == 1
                    % objscalar*vector case
                    for i=1:length(obj)
                        out = obj.data * a;
                    end
                elseif numcols(a) == 1
                    % objvector*scalar case
                    for i=1:length(obj)
                        out(:,i) = obj(i).data * a;
                    end
                else
                    error('RTB:RTBPose:badops', 'unequal vector lengths to * operator');
                end
                
                if obj1.isSE
                    % is SE(n) convert to homogeneous form
                    out = out(1:end-1,:);
                end
            elseif isa(obj, 'SE2') && isa(a, 'polyshape')
                    % special case, planar rigid body transform of a polyshape
                    out = polyshape( (obj * a.Vertices')' );
            elseif isa(obj, 'SE3') && isa(a, 'Plucker')
                A = [obj.R -skew(obj.t); zeros(3,3) obj.R];
                out = A * a;   % invokes mtimes Plucker.mtimes
            else
                error('RTB:RTBPose:badops', 'invalid operand types to * operator');
            end
        end
        
        function out = prod(obj)
            %RTBPose.prod Compound array of poses
            %
            % T.prod is a pose representing the product (composition) of the
            % successive elements of T (1xN).  P is a scalar of the same type (SO2,
            % SE2, SO3, SE3) as T.
            %
            % Note::
            % - Composition is performed with the .* operator.
            %
            % See also RTBPose.times, RTBPose.mtimes.
            out = obj(1);
            
            for i=2:length(obj)
                out = out .* obj(i);
            end
        end
        
        function out = mrdivide(obj, a)
            %RTBPose.mrdivide  Compound SO2 object with inverse
            %
            % R = P/Q is a pose object representing the composition of the pose object P by the
            % inverse of the pose object Q, which is matrix multiplication
            % of their equivalent matrices with the second one inverted.
            %
            % If either, or both, of P or Q are vectors, then the result is a vector.
            %
            % If P is a vector (1xN) then R is a vector (1xN) such that R(i) = P(i)/Q.
            %
            % If Q is a vector (1xN) then R is a vector (1xN) such thatR(i) = P/Q(i).
            %
            % If both P and Q are vectors (1xN) then R is a vector (1xN) such that
            % R(i) = P(i)/R(i).
            %
            % See also RTBPose.mtimes.
            
            obj1 = obj(1);
            n = obj1.dim;
            
            if strcmp(class(obj1), class(a))
                % obj / obj
                out = repmat(obj1, 1, max(length(obj),length(a)));
                if length(obj) == length(a)
                    % do vector/vector and scalar/scalar case
                    for i=1:length(obj)
                        out(i) = obj1.new( obj(i).data * inv(a(i).data));
                    end
                elseif length(obj) == 1
                    % scalar/vector case
                    for i=1:length(a)
                        out(i) = obj1.new( obj.data * inv(a(i).data) );
                    end
                elseif length(a) == 1
                    % vector/scalar case
                    for i=1:length(obj)
                        out(i) = obj1.new( obj(i).data * inv(a.data));
                    end
                else
                    error('RTB:RTBPose:badops', 'unequal vector lengths to / operator');
                end 
            else
                error('RTB:RTBPose:badops', 'invalid operand types to / operator');
            end
        end
  
        function v = subs(obj, old, new)
            %RTBPose.subs Symbolic substitution
            %
            % T = subs(T, old, new) replaces OLD with NEW in the symbolic
            % transformation T.
            %
            % See also: subs

            v = obj.new();  % clone the input
            v.data = subs(v.data, old, new);
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  COMPATABILITY/CONVERSION METHODS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function [R,t] = tr2rt(obj)
            %tr2rt  Split rotational and translational components  (compatibility)
            %
            % [R,t] = tr2rt(P) returns the rotation matrix and translation vector
            % corresponding to the pose P which is either SE2 or SE3.
            %
            % Compatible with matrix function [R,t] = tr2rt(T)
            n = numcols(obj.data);
            
            assert(isSE(obj), 'only applicable to SE2/3 class');
            if length(obj) > 1
                R = zeros(3,3,length(obj));
                t = zeros(length(obj), 3);
                for i=1:length(obj)
                    R(:,:,i) = obj(i).R;
                    t(i,:) = obj(i).t';
                end
            else
                R = obj.R;
                t = obj.t;
            end
        end
        
        function R = t2r(obj)
            %t2r  Get rotation matrix  (compatibility)
            %
            % R = t2r(P) returns the rotation matrix corresponding to the pose P which is either SE2 or SE3.
            %
            % Compatible with matrix function R = t2r(T)
            n = numcols(obj.data);
            
            assert(isSE(obj), 'only applicable to SE2/3 class');
            if length(obj) > 1
                R = zeros(3,3,length(obj));
                for i=1:length(obj)
                    R(:,:,i) = obj(i).R;
                end
            else
                R = obj.R;
            end
        end
        
                
        function d = double(obj)
            %RTBPose.double  Convert to matrix
            %
            % T = P.double() is a matrix representation of the pose P, either a
            % rotation matrix or a homogeneous transformation matrix.
            %
            % If P is a vector (1xN) then T will be a 3-dimensional array (MxMxN).
            %
            % Notes::
            % - If the pose is symbolic the result will be a symbolic matrix.
            
            if ~isa(obj(1).data, 'sym')
                d = zeros( [size(obj(1).data) length(obj)] );
            end
            for i=1:length(obj)
                d(:,:,i) = obj(i).data;
            end
        end
        
        
        function out = trprint(obj, varargin)
            %TRPRINT Compact display of homogeneous transformation  (compatibility)
            %
            % trprint(P, OPTIONS) displays the homogoneous transform in a compact single-line
            % format.  If P is a vector then each element is printed on a separate
            % line.
            %
            % Compatible with matrix function trprint(T).
            %
            % Options (inherited from trprint)::
            % 'rpy'        display with rotation in roll/pitch/yaw angles (default)
            % 'euler'      display with rotation in ZYX Euler angles
            % 'angvec'     display with rotation in angle/vector format
            % 'radian'     display angle in radians (default is degrees)
            % 'fmt', f     use format string f for all numbers, (default %g)
            % 'label',l    display the text before the transform
            %
            % See also RTBPose.print, trprint.
            if nargout == 0
                print(obj, varargin{:});
            else
                out = print(obj, varargin{:});
            end
        end
        
        function out = trprint2(obj, varargin)
            %TRPRINT2 Compact display of homogeneous transformation  (compatibility)
            %
            % trprint2(P, OPTIONS) displays the homogoneous transform in a compact single-line
            % format.  If P is a vector then each element is printed on a separate
            % line.
            %
            % Compatible with matrix function trprint2(T).
            %
            % Options (inherited from trprint2)::
            % 'radian'     display angle in radians (default is degrees)
            % 'fmt', f     use format string f for all numbers, (default %g)
            % 'label',l    display the text before the transform
            %
            % See also RTBPose.print, trprint2.
            if nargout == 0
                print(obj, varargin{:});
            else
                out = print(obj, varargin{:});
            end
            
        end
        
        function trplot(obj, varargin)
            %TRPLOT Draw a coordinate frame (compatibility)
            %
            % trplot(P, OPTIONS) draws a 3D coordinate frame represented by P which is
            % SO2, SO3, SE2, SE3.
            %
            % Compatible with matrix function trplot(T).
            %
            % Options (inherited from trplot)::
            % 'handle',h         Update the specified handle
            % 'color',C          The color to draw the axes, MATLAB colorspec C
            % 'noaxes'           Don't display axes on the plot
            % 'axis',A           Set dimensions of the MATLAB axes to A=[xmin xmax ymin ymax zmin zmax]
            % 'frame',F          The coordinate frame is named {F} and the subscript on the axis labels is F.
            % 'framelabel',F     The coordinate frame is named {F}, axes have no subscripts.
            % 'text_opts', opt   A cell array of MATLAB text properties
            % 'axhandle',A       Draw in the MATLAB axes specified by the axis handle A
            % 'view',V           Set plot view parameters V=[az el] angles, or 'auto'
            %                    for view toward origin of coordinate frame
            % 'length',s         Length of the coordinate frame arms (default 1)
            % 'arrow'            Use arrows rather than line segments for the axes
            % 'width', w         Width of arrow tips (default 1)
            % 'thick',t          Thickness of lines (default 0.5)
            % 'perspective'      Display the axes with perspective projection
            % '3d'               Plot in 3D using anaglyph graphics
            % 'anaglyph',A       Specify anaglyph colors for '3d' as 2 characters for
            %                    left and right (default colors 'rc'): chosen from
            %                    r)ed, g)reen, b)lue, c)yan, m)agenta.
            % 'dispar',D         Disparity for 3d display (default 0.1)
            % 'text'             Enable display of X,Y,Z labels on the frame
            % 'labels',L         Label the X,Y,Z axes with the 1st, 2nd, 3rd character of the string L
            % 'rgb'              Display X,Y,Z axes in colors red, green, blue respectively
            % 'rviz'             Display chunky rviz style axes
            %
            % See also RTBPose.plot, trplot.
            obj.plot(varargin{:});
        end
        
        function trplot2(obj, varargin)
            %TRPLOT2 Draw a coordinate frame (compatibility)
            %
            % trplot2(P, OPTIONS) draws a 2D coordinate frame represented by P
            %
            % Compatible with matrix function trplot2(T).
            %
            % Options (inherited from trplot)::
            % 'handle',h         Update the specified handle
            % 'axis',A           Set dimensions of the MATLAB axes to A=[xmin xmax ymin ymax]
            % 'color', c         The color to draw the axes, MATLAB colorspec
            % 'noaxes'           Don't display axes on the plot
            % 'frame',F          The frame is named {F} and the subscript on the axis labels is F.
            % 'framelabel',F     The coordinate frame is named {F}, axes have no subscripts.
            % 'text_opts', opt   A cell array of Matlab text properties
            % 'axhandle',A       Draw in the MATLAB axes specified by A
            % 'view',V           Set plot view parameters V=[az el] angles, or 'auto'
            %                    for view toward origin of coordinate frame
            % 'length',s         Length of the coordinate frame arms (default 1)
            % 'arrow'            Use arrows rather than line segments for the axes
            % 'width', w         Width of arrow tips
            %
            % See also RTBPose.plot, trplot2.
            obj.plot(varargin{:});
        end
        
        function tranimate(obj, varargin)
            %TRANIMATE Animate a coordinate frame (compatibility)
            %
            % TRANIMATE(P1, P2, OPTIONS) animates a 3D coordinate frame moving from
            % pose P1 to pose P2, which can be SO2, SO3, SE2 or SE3.
            %
            % TRANIMATE(P, OPTIONS) animates a coordinate frame moving from the identity pose
            % to the pose P represented by any of the types listed above.
            %
            % TRANIMATE(PV, OPTIONS) animates a trajectory, where PV is a vector of
            % SO2, SO3, SE2, SE3 objects.
            %
            % Compatible with matrix function tranimate(T), tranimate(T1, T2).
            %
            % Options (inherited from tranimate)::
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
            % See also RTBPose.animate, tranimate.
            
            obj.animate(varargin{:});
        end
        
        function tranimate2(obj, varargin)
            obj.animate(varargin{:});
        end
        
        function v = isrot(obj)
            v = obj.dim == 3 && ~obj.isSE;
        end
        
        function v = isrot2(obj)
            v = obj.dim == 2 && ~obj.isSE;
        end
        
        function v = ishomog(obj)
            v = obj.dim == 4 && obj.isSE;
        end
        
        function v = ishomog2(obj)
            v = obj.dim == 3 && obj.isSE;
        end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%  DISPLAY METHODS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function display(obj)
            %RTBPose.display Display a pose
            %
            % P.display() displays the pose.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is an RTBPose subclass object and the command has no trailing
            %   semicolon.
            % - If the function cprintf is found is used to colorise the matrix,
            %   rotational elements in red, translational in blue.
            %
            % See also SO2, SO3, SE2, SE3.
            
            try  % for Octave
                loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            catch
                loose = 0;
            end
            if loose
                disp(' ');
            end
            
            obj.render(inputname(1));  % the hard work done in render
        end
        
        function disp(obj)
            disp( char(obj) );
        end
        
        function s2 = char(obj)
            %RTBPose.char Convert to string
            %
            % s = P.char() is a string showing homogeneous transformation elements as
            % a matrix.
            %
            % See also RTBPose.display.
            s = num2str(obj.data, '%10.4g'); %num2str(obj.data, 4);
            for i=1:numrows(s);
                s2(i,:) = ['    ', s(i,:)];
            end
        end

        
        function out = print(obj, varargin)
            %RTBPose.print Compact display of pose
            %
            % P.print(OPTIONS) displays the homogoneous transform in a compact single-line
            % format.  If P is a vector then each element is printed on a separate
            % line.
            %
            % Options are passed through to trprint or trprint2 depending on the object
            % type.
            %
            % See also trprint, trprint2.
            if nargout == 0
                for T=obj
                    trprint(T.T, varargin{:});
                end
            else
                out = '';
                
                for T=obj
                    out = strvcat(out, trprint(T.T, varargin{:}));
                end
            end
        end
        
        function animate(obj, varargin)
            %RTBPose.animate Animate a coordinate frame
            %
            % RTBPose.animate(P1, P2, OPTIONS) animates a 3D coordinate frame moving from
            % pose P1 to pose P2, which can be SO3 or SE3.
            %
            % RTBPose.animate(P, OPTIONS) animates a coordinate frame moving from the identity pose
            % to the pose P represented by any of the types listed above.
            %
            % RTBPose.animate(PV, OPTIONS) animates a trajectory, where PV is a vector of
            % SO2, SO3, SE2, SE3 objects.
            %
            % Compatible with matrix function tranimate(T), tranimate(T1, T2).
            %
            % Options (inherited from tranimate)::
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
            % See also tranimate.
            
            % invoke classic functions
            if length(varargin) > 0 && isa(varargin{1}, 'RTBPose')
                % tranimate(T1, T2, args)
                switch class(obj)
                    case 'SO2'
                        tranimate2(obj.R, varargin{1}.R, varargin{2:end});
                        
                    case 'SE2'
                        tranimate2(obj.T, varargin{1}.T, varargin{2:end});
                        
                    case 'SO3'
                        tranimate(obj.R, varargin{1}.R, varargin{2:end});
                        
                    case 'SE3'
                        tranimate(obj.T, varargin{1}.T, varargin{2:end});
                end
            else
                % tranimate(T1, args)
                switch class(obj)
                    case 'SO2'
                        tranimate2(obj.R, varargin{:});
                        
                    case 'SE2'
                        tranimate2(obj.T, varargin{:});
                        
                    case 'SO3'
                        tranimate(obj.R, varargin{:});
                        
                    case 'SE3'
                        tranimate(obj.T, varargin{:});
                end
            end
            

        end
        
        function varargout = plot(obj, varargin)
            %TRPLOT Draw a coordinate frame (compatibility)
            %
            % trplot(P, OPTIONS) draws a 3D coordinate frame represented by P which is
            % SO2, SO3, SE2 or SE3.
            %
            % Compatible with matrix function trplot(T).
            %
            % Options are passed through to trplot or trplot2 depending on the object
            % type.
            %
            % See also trplot, trplot2.
             
             
            switch class(obj)
                case 'SO2'
                    [varargout{1:nargout}] = trplot2(obj.R, varargin{:});
                    
                case 'SE2'
                    [varargout{1:nargout}] = trplot2(obj.T, varargin{:});
                    
                case 'SO3'
                    [varargout{1:nargout}] = trplot(obj.R, varargin{:});
                    
                case 'SE3'
                    [varargout{1:nargout}] = trplot(obj.T, varargin{:});
            end
        end
        
        
    end
    
    methods (Access=private)
        function render(obj, varname)
            
            if isa(obj(1).data, 'sym')
                % use MATLAB default disp() function for symbolic object
                disp(obj.data);
            else
                % else render the elements with specified format and color
                fmtR = '%10.4f';
                fmtt = '%10.4g';
                fmt0 = '%10.0f';
                if exist('cprintf')
                    print = @(color, fmt, value) cprintf(color, fmt, value);
                else
                    print = @(color, fmt, value) fprintf(fmt, value);
                end
                switch class(obj)
                    case 'SO2',  nr = 2; nt = 0;
                    case 'SE2',  nr = 2; nt = 1;
                    case 'SO3',  nr = 3; nt = 0;
                    case 'SE3',  nr = 3; nt = 1;
                end
                
                for i=1:length(obj)
                    M = obj(i).data;
                    if length(obj) > 1
                        fprintf('\n%s(%d) = \n', varname, i);
                    else
                        fprintf('\n%s = \n', varname);
                        
                    end
                    M(abs(M)<1000*eps) = 0;
                    
                    for row=1:nr
                        for col=1:(nr+nt)
                            if col <= nr
                                % rotation matrix
                                v = M(row,col);
                                
                                if fix(v) == v
                                    print('Errors', fmt0, v); % red
                                else
                                    print('Errors', fmtR, v); % red
                                end
                            else
                                % translation
                                print('Keywords', fmtt, M(row,col)); % blue
                            end
                        end
                        fprintf('\n');
                    end
                    % last row
                    if nt > 0
                        for col=1:(nr+nt)
                            print('Text', fmt0, M(nr+nt,col));
                        end
                        fprintf('\n');
                    end
                end
            end
        end
    end
    
end
