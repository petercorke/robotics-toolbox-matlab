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
function name_ = models(query)
    
    path = fileparts( which('rotx') );
    path = fullfile(path, 'models');
    
    models = dir(fullfile(path, 'mdl_*.m'));
    info = {};
    name = {};
    for model=models'
        fid = fopen( fullfile(path, model.name), 'r');
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
    
    % now sort and print the matching models
    for i = sort(info)'
        fprintf('%s\n', i{1});
    end
    
    % optionally return a list of model names
    if nargout == 1
        name_ = name;
    end
    
end
