%CODEGENERATOR.LOGMSG Print CodeGenerator logs.
%
% count = CGen.logmsg( FORMAT, A, ...) is the number of characters written to the CGen.logfile.
% For the additional arguments see fprintf.
%
% Note::
%  Matlab ships with a function for writing formatted strings into a text
%  file or to the console (fprintf). The function works with single 
%  target identifiers (file, console, string). This function uses the 
%  same syntax as for the fprintf function to output log messages to 
%  either the Matlab console, a log file or both. 
%
% Authors::
%  Joern Malzahn   
%  2012 RST, Technische Universitaet Dortmund, Germany
%  http://www.rst.e-technik.tu-dortmund.de
%
%  See also multidfprintf,fprintf,sprintf.

% Copyright (C) 1993-2012, by Peter I. Corke
% Copyright (C) 2012-2013, by Joern Malzahn
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
%
% http://www.petercorke.com
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

function [cnt] = logmsg(CGen, varargin)

% Output to logfile?
if ~isempty(CGen.logfile)
    logfid = fopen(CGen.logfile,'a+');
else
    logfid = [];
end

% write message to multiple destinations
cnt = multidfprintf([CGen.verbose, logfid],varargin{:});

% Logfile to close?
if ~isempty(CGen.logfile)
    logfid = fclose(logfid);
end