%TWIST SE(2) and SE(3) Twist class
%
% A Twist class holds the parameters of a twist, a representation of a
% rigid body displacement in SE(2) or SE(3).
%
% Methods::
%  S             twist vector (1x3 or 1x6)
%  se            twist as (augmented) skew-symmetric matrix (3x3 or 4x4)
%  T             convert to homogeneous transformation (3x3 or 4x4)
%  R             convert rotational part to matrix (2x2 or 3x3)
%  exp           synonym for T
%  ad            logarithm of adjoint
%  pitch         pitch of the screw, SE(3) only
%  pole          a point on the line of the screw
%  prod          product of a vector of Twists
%  theta         rotation about the screw
%  line          Plucker line object representing line of the screw
%  display       print the Twist parameters in human readable form
%  char          convert to string
%
% Conversion methods::
%  SE            convert to SE2 or SE3 object
%  double        convert to real vector
%
% Overloaded operators::
%  *             compose two Twists
%  *             multiply Twist by a scalar
%
% Properties (read only)::
%  v             moment part of twist (2x1 or 3x1)
%  w             direction part of twist (1x1 or 3x1)
%
% References::
% - "Mechanics, planning and control"
%   Park & Lynch, Cambridge, 2016.
%
% See also trexp, trexp2, trlog.

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

classdef Twist
    properties (SetAccess = protected)
        v  %axis direction (column vector)
        w  %moment (column vector)
    end
    
    methods
        function tw = Twist(T, varargin)
        %Twist.Twist Create Twist object
        %
        % TW = Twist(T) is a Twist object representing the SE(2) or SE(3)
        % homogeneous transformation matrix T (3x3 or 4x4).
        %
        % TW = Twist(V) is a twist object where the vector is specified directly.
        %
        % 3D CASE::
        %
        % TW = Twist('R', A, Q) is a Twist object representing rotation about the
        % axis of direction A (3x1) and passing through the point Q (3x1).
        %
        % TW = Twist('R', A, Q, P) as above but with a pitch of P (distance/angle).
        %
        % TW = Twist('T', A) is a Twist object representing translation in the
        % direction of A (3x1).
        %
        % 2D CASE::
        %
        % TW = Twist('R', Q) is a Twist object representing rotation about the point Q (2x1).
        %
        % TW = Twist('T', A) is a Twist object representing translation in the
        % direction of A (2x1).
        %
        % Notes::
        %  The argument 'P' for prismatic is synonymous with 'T'.

            if ischar(T)
                % 'P', dir
                % 'R', dir, point 3D
                % 'R', point   2D
                switch upper(T)
                    case 'R'
                        if nargin == 2
                            % 2D case
                            
                            point = varargin{1};
                            point = point(:);
                            v = -cross([0 0 1]', [point; 0]);
                            w = 1;
                            v = v(1:2);
                        else
                            % 3D case
                            dir = varargin{1};
                            if length(dir) < 3
                                error('RTB:Twist:badarg', 'For 2d case can only specify position');
                            end
                            point = varargin{2};
                            
                            w = unit(dir(:));
                            
                            v = -cross(w, point(:));
                            if nargin >= 4
                                pitch = varargin{3};
                                v = v + pitch * w;
                            end
                        end
                        
                    case {'P', 'T'}
                        dir = varargin{1};
                        
                        if length(dir) == 2
                            w = 0;
                        else
                            w = [0 0 0]';
                        end
                        v = unit(dir(:));
                end
                
                if ~isa(v, 'sym')
                    v(abs(v)<eps) = 0;
                end
                if ~isa(w, 'sym')
                    w(abs(w)<eps) = 0;
                end
                tw.v = v;
                tw.w = w;
            elseif numrows(T) == numcols(T)
                % it's a square matrix
                if T(end,end) == 1
                    % its a homogeneous matrix, take the logarithm
                    if numcols(T) == 4
                        S = trlog(T);  % use closed form for SE(3)
                    else
                        S = logm(T);
                    end
                    [skw,v] = tr2rt(S);
                    tw.v = v;
                    tw.w = vex(skw);
                else
                    % it's an augmented skew matrix, unpack it
                    [skw,v] = tr2rt(T);
                    tw.v = v;
                    tw.w = vex(skw);
                end
            elseif isvector(T)
                % its a row vector form of twist, unpack it
                switch length(T)
                    case 3
                        tw.v = T(1:2)'; tw.w = T(3);
                        
                    case 6
                        tw.v = T(1:3)'; tw.w = T(4:6)';
                        
                    otherwise
                        error('RTB:Twist:badarg', '3 or 6 element vector expected');
                end
            end
        end
        
        function Su = unit(S)
        %Twist.unit Return a unit twist
        %
        % TW.unit() is a Twist object representing a unit aligned with the Twist
        % TW.
            if abs(S.w) > 10*eps
                % rotational twist
                Su = Twist( double(S) / norm(S.w) );
            else
                % prismatic twist
                Su = Twist( [unit(S.v); 0; 0; 0] );
            end
        end
        
        function x = S(tw)
        %Twist.S Return the twist vector
        %
        % TW.S is the twist vector in se(2) or se(3) as a vector (3x1 or 6x1).
        %
        % Notes::
        % - Sometimes referred to as the twist coordinate vector.
            x = [tw.v; tw.w];
        end
        
        function x = double(tw)
        %Twist.double Return the twist vector
        %
        % double(TW) is the twist vector in se(2) or se(3) as a vector (3x1 or
        % 6x1). If TW is a vector (1xN) of Twists the result is a matrix (6xN) with
        % one column per twist.
        %
        % Notes::
        % - Sometimes referred to as the twist coordinate vector.
            x = [tw.v; tw.w];
        end
        
        function x = se(tw)
        %Twist.se Return the twist matrix
        %
        % TW.se is the twist matrix in se(2) or se(3) which is an augmented
        % skew-symmetric matrix (3x3 or 4x4).
        %
        x = skewa(tw.S);
        end

        
        function c = mtimes(a, b)
        %Twist.mtimes Multiply twist by twist or scalar
        %
        % TW1 * TW2 is a new Twist representing the composition of twists TW1 and
        % TW2.
        %
        % TW * T is an SE2 or SE3 that is the composition of the twist TW and the
        % homogeneous transformation object T.
        %
        % TW * S with its twist coordinates scaled by scalar S.
        %
        % TW * T compounds a twist with an SE2/3 transformation
        %
            
            if isa(a, 'Twist')
                if isa(b, 'Twist')
                    % twist composition
                    c = Twist( a.exp * b.exp);
                elseif length(a.v) == 2 && ishomog2(b)
                    % compose a twist with SE2, result is an SE3
                    c = SE2(a.T * double(b));
                elseif length(a.v) == 3 && ishomog(b)
                    % compose a twist with SE2, result is an SE3
                    c = SE3(a.T * double(b));
                else
                    error('RTB:Twist', 'twist * SEn, operands don''t conform');
                end
            elseif isreal(a) && isa(b, 'Twist')
                c = Twist(a * b.S);
            elseif isa(a, 'Twist') && isreal(b)
                c = Twist(a.S * b);
            else
                error('RTB:Twist: incorrect operand types for * operator')
            end
        end
                
        function x = exp(tw, varargin)
        %Twist.exp Convert twist to homogeneous transformation
        %
        % TW.exp is the homogeneous transformation equivalent to the twist (SE2 or SE3).
        %
        % TW.exp(THETA) as above but with a rotation of THETA about the twist.
        %
        % Notes::
        % - For the second form the twist must, if rotational, have a unit rotational component.
        %
        % See also Twist.T, trexp, trexp2.
            opt.deg = false;
            [opt,args] = tb_optparse(opt, varargin);

            if opt.deg && all(tw.w == 0)
                warning('Twist: using degree mode for a prismatic twist');
            end

            if length(args) > 0
                theta = args{1};

                if opt.deg
                    theta = theta * pi/180;
                end
            else
                theta = 1;
            end

            ntheta = length(theta);
            assert(length(tw) == ntheta || length(tw) == 1, 'Twist:exp:badarg', 'length of twist vector must be 1 or length of theta vector')
            if length(tw(1).v) == 2
                x(ntheta) = SE2;
                if length(tw) == ntheta
                    for i=1:ntheta
                        x(i) = trexp2( tw(i).S * theta(i) );
                    end
                else
                    for i=1:ntheta
                        x(i) = trexp2( tw.S * theta(i) );
                    end
                end
            else
                x(ntheta) = SE3;
                if length(tw) == ntheta
                    for i=1:ntheta
                        x(i) = trexp( tw(i).S * theta(i) );
                    end
                else
                    for i=1:ntheta
                        x(i) = trexp( tw.S * theta(i) );
                    end
                end
            end
        end
        
        function x = ad(tw)
        %Twist.ad Logarithm of adjoint
        %
        % TW.ad is the logarithm of the adjoint matrix of the corresponding
        % homogeneous transformation.
        %
        % See also SE3.Ad.
            x = [ skew(tw.w) skew(tw.v); zeros(3,3) skew(tw.w) ];
        end
        
        function x = Ad(tw)
        %Twist.Ad Adjoint
        %
        % TW.Ad is the adjoint matrix of the corresponding
        % homogeneous transformation.
        %
        % See also SE3.Ad.
            x = tw.SE.Ad;
        end
        
        
        function out = SE(tw)
        %Twist.SE Convert twist to SE2 or SE3 object
        %
        % TW.SE is an SE2 or SE3 object representing the homogeneous transformation equivalent to the twist.
        %
        % See also Twist.T, SE2, SE3.
            if length(tw.v) == 2
                out = SE2( tw.T );
            else
                out = SE3( tw.T );
            end
        end 
                
        function x = T(tw, varargin)
        %Twist.T Convert twist to homogeneous transformation
        %
        % TW.T is the homogeneous transformation equivalent to the twist (3x3 or 4x4).
        %
        % TW.T(THETA) as above but with a rotation of THETA about the twist.
        %
        % Notes::
        % - For the second form the twist must, if rotational, have a unit rotational component.
        %
        % See also Twist.exp, trexp, trexp2, trinterp, trinterp2.
            x = double( tw.exp(varargin{:}) );
        end
        
        
        function p = pitch(tw)
        %Twist.pitch Pitch of the twist
        %
        % TW.pitch is the pitch of the Twist as a scalar in units of distance per radian.
        %
        % Notes::
        % - For 3D case only.
        
        if length(tw.v) == 2
                p = 0;
            else
                p = tw.w' * tw.v;
            end
        end
        
        function L = line(tw)
        %Twist.line Line of twist axis in Plucker form
        %
        % TW.line is a Plucker object representing the line of the twist axis.
        %
        % Notes::
        % - For 3D case only.
        %
        % See also Plucker.
        
                % V = -tw.v - tw.pitch * tw.w;
                for i=1:length(tw)
                    L(i) = Plucker('wv', tw(i).w, -tw(i).v - tw(i).pitch * tw(i).w);
                end
        end
        
        function out = prod(obj)
            %Twist.prod Compound array of twists
            %
            % TW.prod is a twist representing the product (composition) of the
            % successive elements of TW (1xN), an array of Twists.
            %
            %
            % See also RTBPose.prod, Twist.mtimes.
            out = obj(1);
            
            for i=2:length(obj)
                out = out .* obj(i);
            end
        end
        
        function p = pole(tw)
        %Twist.pole Point on the twist axis
        %
        % TW.pole is a point on the twist axis (2x1 or 3x1).
        %
        % Notes::
        % - For pure translation this point is at infinity.
        if length(tw.v) == 2
                v = [tw.v; 0];
                w = [0 0 tw.w]';
                p = cross(w, v) / tw.theta();
                p = p(1:2);
            else
                p = cross(tw.w, tw.v) / tw.theta();
            end
        end
        
        function th = theta(tw)
        %Twist.theta Twist rotation
        %
        % TW.theta is the rotation (1x1) about the twist axis in radians.
        %

            th = norm(tw.w);
        end
        
            
        function s = char(tw)
        %Twist.char Convert to string
        %
        % s = TW.char() is a string showing Twist parameters in a compact single line format.
        % If TW is a vector of Twist objects return a string with one line per Twist.
        %
        % See also Twist.display.
        s = '';
            for i=1:length(tw)
                
                ps = '( ';
                ps = [ ps, sprintf('%0.5g  ', tw(i).v) ];
                ps = [ ps(1:end-2), '; '];
                ps = [ ps, sprintf('%0.5g  ', tw(i).w) ];
                ps = [ ps(1:end-2), ' )'];
                if isempty(s)
                    s = ps;
                else
                    s = char(s, ps);
                end
            end
            

        end
        
        function display(tw)
            %Twist.display Display parameters
            %
            % L.display() displays the twist parameters in compact single line format.  If L is a
            % vector of Twist objects displays one line per element.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Twist object and the command has no trailing
            %   semicolon.
            %
            % See also Twist.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(tw) );
        end % display()

    end
end