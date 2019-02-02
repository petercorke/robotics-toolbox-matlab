%MODELS Summarise and search available robot models
%
% MODELS() lists keywords associated with each of the models in Robotics Toolbox.
%
% MODELS(QUERY) lists those models that match the keyword QUERY.  Case is
% ignored in the comparison.
%
% M = MODELS(QUERY) as above but returns a cell array (Nx1) of the names of the
% M-files that define the models.
%
% Examples::
%         models
%         models('modified_DH')  % all models using modified DH notation
%         models('kinova')       % all Kinova robot models
%         models('6dof')         % all 6dof robot models
%         models('redundant')    % all redundant robot models, >6 DOF
%         models('prismatic')    % all robots with a prismatic joint
%
% Notes::
% - A model is a file mdl_*.m in the models folder of the RTB directory.
% - The keywords are indicated by a line '% MODEL: ' after the main comment
%   block.
%

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
function name_ = models(query)
    
    % first find the path to the models
    pth = mfilename('fullpath');
    pth = fileparts(pth);
    pth = fullfile(pth, 'models');
    assert(exist(pth, 'dir') > 0, 'RTB:models:nomodel', 'models folder not found');

    models = dir(fullfile(pth, 'mdl_*.m'));
    info = {};
    name = {};
    for model=models'
        fid = fopen( fullfile(pth, model.name), 'r');
        while true
            line = fgetl(fid);
            if line == -1
                break;
            end
            if (length(line) > 11) && (strcmp(line(1:11), '% Copyright:') == 1)
                % an empty line, done with the header block
                break;
            end
            [p,n,e] = fileparts(model.name);
            
            if (length(line) > 8) && (strcmp(line(1:8), '% MODEL:') == 1)
                % we have a model description line
                
                line = line(10:end);  % trim off the tag
                
                if nargin > 0
                    % we have a  query
                    if strfind(lower(line), lower(query))
                        info = [info; [line ' (' n ')' ]];
                        name = [name; n];
                    end
                else
                    % no query, report all
                    info = [info; [line ' (' n ')' ]];
                    name = [name n];
                end
                break
            end
        end
        fclose(fid);
    end
    

    
    % optionally return a list of model names
    if nargout == 1
        name_ = name';
    else
        % now sort and print the matching models
        for i = sort(info)'
            fprintf('%s\n', i{1});
        end
    end
    
end
