%ROBOT	robot object constructor
%
%	ROBOT
%	ROBOT(robot)		create a copy of an existing ROBOT object
%	ROBOT(LINK, ...)	create from a cell array of LINK objects
%	ROBOT(DH, ...)		create from legacy DYN matrix
%	ROBOT(DYN, ...)		create from legacy DYN matrix
%
%	optional trailing arguments are:
%		Name			robot type or name
%		Manufacturer		who built it
%		Comment			general comment
%
%  If the legacy matrix forms are used the default name is the workspace
% variable that held the data.

% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%
% Copyright (C) 1993-2011, by Peter I. Corke
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
function r = SerialLink(L, varargin)
	r.name = 'noname';
	r.manufacturer = '';
	r.comment = '';
	r.links = [];
	r.n = 0;
	r.mdh = 0;
	r.gravity = [0; 0; 9.81];
	r.base = eye(4,4);
	r.tool = eye(4,4);
	r.handle = [];  % graphics handles
	r.q = [];   % current joint angles

	r.lineopt = {'Color', 'black', 'Linewidth', 4};
	r.shadowopt = {'Color', 0.7*[1 1 1], 'Linewidth', 3};
	r.plotopt = {};
	r = class(r, 'SerialLink');
	
	if nargin == 0
	%zero argument constructor 
		return;
		
	else
		if isa(L, 'SerialLink')
			if length(L) == 1
				% clone the passed robot
				for j=1:L.n
					newlinks(j) = L.links(j);
				end
				r.links = newlinks;
			else
				% compound the robots in the vector
				r = L(1);
				for k=2:length(L)
					r = r * L(k);
				end
			end
		elseif isa(L,'Link')
			r.links = L; %attach the links
			
		elseif isa(L, 'double')
			% legacy matrix
			dh_dyn = L;
			clear L
			for j=1:numrows(dh_dyn)
				L(j) =        Link();
				L(j).theta =  dh_dyn(j,1);
				L(j).d =      dh_dyn(j,2);
				L(j).a =      dh_dyn(j,3);
				L(j).alpha =  dh_dyn(j,4);

				if numcols(dh_dyn) > 4
					L(j).sigma = dh_dyn(j,5);
				end
			end
			r.links = L;
		else
			error('unknown type passed to SerialLink');
		end
		r.n = length(r.links);
	end
	%process the rest of the arguments in key, value pairs
	opt.name = 'robot';
	opt.comment = [];
	opt.manufacturer = [];
	opt.base = [];
	opt.tool = [];
	opt.offset = [];
	opt.qlim = [];
	opt.plotopt = [];

	[opt,out] = tb_optparse(opt, varargin);
	if ~isempty(out)
		error( sprintf('unknown option <%s>', out{1}));
	end

	% copy the properties to robot object
            p = fieldnames(r);
            for i=1:length(p)
                if isfield(opt, p{i}) && ~isempty(getfield(opt, p{i}))
                    
					r = setfield(r, p{i}, getfield(opt, p{i}));
					
				end
            end

				
	% set the robot object mdh status flag
	mdh = [r.links.mdh];
	if all(mdh == 0)
		r.mdh = mdh(1);
	elseif all (mdh == 1)
		r.mdh = mdh(1);
	else
		error('SerialLink has mixed D&H links conventions');
	end
	r = class(r, 'SerialLink');

endfunction
