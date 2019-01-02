%GETPROFILEFUNCTIONSTATS Summary of this function goes here
% Detailed explanation goes here
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)

% Copyright (C) 2012-2018, by Joern Malzahn
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

function [ funstats] = getprofilefunctionstats( pstats , desfun, varargin)

nEl = numel(pstats.FunctionTable);
desfname = which(desfun);
funstats = [];

if isempty(desfname)
    error(['Function ', desfun, ' not found!']);
end

funtype = '';
if nargin == 3
    switch lower(varargin{1})
        case 'm-function'
        case 'mex-function'
        otherwise
            error('funtype must be either ''M-function'' or ''MEX-function''!');
        end
else
    desfun = [desfun '.m'];
end


for iEl = 1:nEl
    curstats = pstats.FunctionTable(iEl);
    
    if string(curstats.FileName).endsWith(desfun)
        funstats = curstats;
        return
    end
end

end

