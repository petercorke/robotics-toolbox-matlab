function tests = RTBPoseTest
  tests = functiontests(localfunctions);
end
% we will assume that the primitives rotx,trotx, etc. all work


function constructor(testCase)
    q = Quaternion([1 2 3 4]);
    verifyEqual(testCase, q.double(),[1 2 3 4],'absTol',1e-4);
    
    % native
end

function staticconstructors(testCase)
end


function multiply(testCase)
    % scalar x scalar
    
        % vector x vector
        
            % scalar x vector
            
                % vector x scalar
end

function divide(testCase)
    
        % scalar x scalar
    
        % vector x vector
        
            % scalar x vector
            
                % vector x scalar
end

function angle(testCase)
end

function conversions(testCase)
end

