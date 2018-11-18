%% This is for testing the SerialLink models in the robotics Toolboxg
function tests = SerialLinkModelsTest
  tests = functiontests(localfunctions);
end

function models_test(tc)
    models
    
    names = models();
    verifyTrue(tc, iscell(names) );

        
    models('6dof')
    
    names = models('6dof');
    verifyTrue(tc, iscell(names) );
end

function all_models_test(tc)
    
    all = models();
    
    for model = all'
        try
            eval( [model{1} ';'] );
        catch
            verifyFail(tc, sprintf('model %s failed to load', model{1}) );
        end
        
        % find all the SerialLink objects
        vars = whos;
        k = find( cellfun(@(s) strcmp(s,'SerialLink'), {vars.class}) );
        if isempty(k)
            verifyFail(tc, sprintf('no SerialLink models loaded by %s', model{1}) );
        end
        
        for kk = k
            clear(vars(kk).name)
        end
            
%         if exist('qz', 'var') ~= 1
%             verifyFail(tc, sprintf('model %s doesn''t set qz', model{1}));
%         end
        clear qz
    end
end

function query_test(tc)
    names = models('6dof');
    for model = names'
        % attempt to load the model
        try
            eval( [model{1} ';'] );
        catch
            verifyFail(tc, 'model %s failed', model{1});
        end
        
        % find the number of DOF in the model
        vars = whos;
        
        k = find( cellfun(@(s) strcmp(s,'SerialLink'), {vars.class}) );
        
        if ~isempty(k)
            if eval( [vars(k).name '.n'] ) ~= 6
                verifyFail(tc, 'query failed');
            end
        end
        
        clear(vars(k).name)

    end
end