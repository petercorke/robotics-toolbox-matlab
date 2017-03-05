%TRPRINT2 Compact display of SE2 homogeneous transformation
%
% TRPRINT2(T, OPTIONS) displays the homogoneous transform in a compact 
% single-line format.  If T is a homogeneous transform sequence then each 
% element is printed on a separate line.
%
% S = TRPRINT2(T, OPTIONS) as above but returns the string.
%
% TRPRINT T  is the command line form of above, and displays in RPY format.
%
% Options::
% 'radian'     display angle in radians (default is degrees)
% 'fmt', f     use format string f for all numbers, (default %g)
% 'label',l    display the text before the transform
%
% Examples::
%        >> trprint2(T2)
%        t = (0,0), theta = -122.704 deg
%
%
% See also TRPRINT.



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

function out = trprint2(T, varargin)
    
    if ischar(T)
        % command form: trprint T
        trprint( evalin('base', T) );
        return;
    end

    opt.fmt = [];
    opt.radian = false;
    opt.label = '';

    [opt,args] = tb_optparse(opt, varargin);

    s = '';

    if size(T,3) == 1
        if isempty(opt.fmt)
            opt.fmt = '%.3g';
        end
        s = tr2s(T, opt, args{:});
    else
        if isempty(opt.fmt)
            opt.fmt = '%8.2g';
        end        
        
        for i=1:size(T,3)
            % for each 4x4 transform in a possible 3D matrix
            s = char(s, tr2s(T(:,:,i), opt) );
        end
    end

    % if no output provided then display it
    if nargout == 0
        disp(s);
    else
        out = s;
    end
end

function s = tr2s(T, opt)
    % print the translational part if it exists
    if ~isempty(opt.label)
        s = sprintf('%8s: ', opt.label);
    else
        s = '';
    end
    if ~isrot2(T)
        s = strcat(s, sprintf('t = (%s),', vec2s(opt.fmt, transl2(T)')));
    end

    % print the angular part
    ang = atan2(T(2,1), T(1,1));
    if opt.radian
        s = strcat(s, ...
            sprintf(' %s rad', vec2s(opt.fmt, ang)) );
    else
        s = strcat(s, ...
            sprintf(' %s deg', vec2s(opt.fmt, ang*180.0/pi)) );
    end
end

function s = vec2s(fmt, v)
    s = '';
    for i=1:length(v)
        if abs(v(i)) < 1000*eps
            v(i) = 0;
        end
        s = [s, sprintf(fmt, v(i))];
        if i ~= length(v)
            s = [s, ', ']; % don't use strcat, removes trailing spaces
        end
    end
end
