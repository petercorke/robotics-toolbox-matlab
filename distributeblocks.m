%DISTRIBUTEBLOCKS Distribute blocks in Simulink block library
%
% distributeBlocks(MODEL) equidistantly distributes blocks in a Simulink 
% block library named MODEL.
%
% Notes::
% - The MATLAB functions to create Simulink blocks from symbolic
%   expresssions actually place all blocks on top of each other. This
%   function scans a simulink model and rearranges the blocks on an
%   equidistantly spaced grid.
% - The Simulink model must already be opened before running this
%   function!
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de) 
%
% See also symexpr2slblock, doesblockexist.

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

function [  ] = distributeblocks( mdlName )
 
%% Get a list of all first level blocks:
blockNames = find_system(mdlName,'SearchDepth',1);
blockNames = blockNames(2:end); % First cell contains the model name

%% Determine the maximum width and height of a block
allPosC = get_param(blockNames,'Position');
allPosM = cell2mat(allPosC); % [left top right bottom] The maximum value for a coordinate is 32767

widths  = allPosM(:,3)-allPosM(:,1);
heights = allPosM(:,4)-allPosM(:,2);
maxWidth = max(widths);
maxHeight = max(heights);

%% Set grid spacing
wBase = 2*maxWidth;
hBase = 2*maxHeight;

%% Start rearranging blocks
nBlocks = size(allPosM,1);
nColRow = ceil(sqrt(nBlocks));

for iBlocks = 1:nBlocks
    
    % block location on grid
    [row,col] = ind2sub([nColRow, nColRow],iBlocks);
    
    % compute block coordinates
    left = col*wBase;
    right = left+maxWidth;
    top = row*hBase;
    bottom = top+maxHeight;
    
    % apply new coordinates
    set_param(blockNames{iBlocks},'Position',[left top right bottom])
    
end


end

