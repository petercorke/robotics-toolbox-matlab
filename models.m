%MODELS Summarise and search available robot models
%
% models lists keywords associated with each of the models in RTB.
%
% models(query) lists those models that match the keyword QUERY.  Case is
% ignored in the comparison.
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
% - A model is a file mdl_*.m in the top-level RTB directory.
% - The keywords are indicated by a line '% MODEL: ' after the main comment
%   block.
% 
function models(query)

    p = what('robot');
    models = dir(fullfile(p(1).path, 'mdl_*.m'));
    info = {};
    for model=models'
        fid = fopen(model.name, 'r');
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
                line = line(10:end);
                if nargin > 0
                    if strfind(lower(line), lower(query))
                        fprintf('%-16s %s\n', [n ':'], line);
                    end
                else
                    info = [info; [line ' (' n ')' ]];
                end
                break
            end
        end
        fclose(fid);
    end

    if nargin == 0
            for i = sort(info)'
                fprintf('%s\n', i{1});
            end
    end

    end
