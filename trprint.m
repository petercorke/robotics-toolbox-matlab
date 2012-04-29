%TRPRINT Compact display of homogeneous transformation
%
% TRPRINT(T, OPTIONS) displays the homogoneous transform in a compact 
% single-line format.  If T is a homogeneous transform sequence then each 
% element is printed on a separate line.
%
% S = TRPRINT(T, OPTIONS) as above but returns the string.
%
% TRPRINT T  is the command line form of above, and displays in RPY format.
%
% Options::
% 'rpy'        display with rotation in roll/pitch/yaw angles (default)
% 'euler'      display with rotation in ZYX Euler angles
% 'angvec'     display with rotation in angle/vector format
% 'radian'     display angle in radians (default is degrees)
% 'fmt', f     use format string f for all numbers, (default %g)
% 'label',l    display the text before the transform
%
% Examples::
%        >> trprint(T2)
%        t = (0,0,0), RPY = (-122.704,65.4084,-8.11266) deg
%
%        >> trprint(T1, 'label', 'A')
%               A:t = (0,0,0), RPY = (-0,0,-0) deg
%
% See also TR2EUL, TR2RPY, TR2ANGVEC.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function out = trprint(T, varargin)
    
    if ischar(T)
        % command form: trprint T
        trprint( evalin('base', T) );
        return;
    end

    opt.fmt = [];
    opt.mode = {'rpy', 'euler', 'angvec'};
    opt.radian = false;
    opt.label = '';

    opt = tb_optparse(opt, varargin);

    s = '';

    if size(T,3) == 1
        if isempty(opt.fmt)
            opt.fmt = '%g';
        end
        s = tr2s(T, opt);
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
    if ~isrot(T)
        s = strcat(s, sprintf('t = (%s),', vec2s(opt.fmt, transl(T)')));
    end

    % print the angular part in various representations
    switch (opt.mode)
        case {'rpy', 'euler'}
            % angle as a 3-vector
            if strcmp(opt.mode, 'rpy')
                ang = tr2rpy(T);
                label = 'RPY';
            else
                ang = tr2eul(T);
                label = 'EUL';
            end
            if opt.radian
                s = strcat(s, ...
                    sprintf(' %s = (%s) rad', label, vec2s(opt.fmt, ang)) );
            else
                s = strcat(s, ...
                    sprintf(' %s = (%s) deg', label, vec2s(opt.fmt, ang*180.0/pi)) );
            end
        case 'angvec'
            % as a vector and angle
            [th,v] = tr2angvec(T);
            if isnan(v(1))
                s = strcat(s, sprintf(' R = nil') );
            elseif opt.radian
                s = strcat(s, sprintf(' R = (%sdeg | %s)', ...
                    sprintf(opt.fmt, th), vec2s(opt.fmt, v)) );
            else
                s = strcat(s, sprintf(' R = (%sdeg | %s)', ...
                    sprintf(opt.fmt, th*180.0/pi), vec2s(opt.fmt,v)) );
            end
    end
end

function s = vec2s(fmt, v)
    s = '';
    for i=1:length(v)
        s = strcat(s, sprintf(fmt, v(i)));
        if i ~= length(v)
            s = strcat(s, ', ');
        end
    end
end
