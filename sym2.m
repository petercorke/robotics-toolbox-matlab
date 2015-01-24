%SYM2 Subclass of sym class
%
% This is ugly.  The provided sym class can only generate MATLAB functions, not
% expressions.  It can generate expressions in C and Fortran however.
%
% The only way to access this capability is direct to the MuPad engine, and since
% we can't change the sym class we use a subclass and add a matgen method

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

classdef sym2 < sym
    
    methods
        
        function s = sym2(varargin)
            s = s@sym(varargin{:});
        end
        
        function f = matgen(s,varargin)
            %MATGEN  MATLAB representation of a symbolic expression.
            %   MATGEN(S) is a fragment of MATLAB that evaluates symbolic expression S.
            %
            %
            %   See also SYM/PRETTY, SYM/LATEX, SYM/CCODE.
            %
            % Based on sym.fortran().
            
            %   Copyright 1993-2012 The MathWorks, Inc.
            
            t = inputname(1);

            if isempty(t), t = 'T'; end
            if builtin('numel',s) ~= 1, s = normalizesym(s); end
            r = sym(t);
            cleanup = onCleanup(@() mupadmex(['delete ' t ';'],0));
            mupadmex([t ':=' s.s ';']);

                f = sprintf(mupadmex('generate::MATLAB', t, 0));
                f(f == '"') = [];
                f = deblank(f);
        end
    end
end
