function tests = RTBPoseTest
  tests = functiontests(localfunctions);
  clc
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
    % double
    %  tr2rt       convert to rotation matrix and translation vector
%  t2r         convert to rotation matrix
%  trprint     print single line representation
%  trprint2    print single line representation
%  trplot      plot coordinate frame
%  trplot2     plot coordinate frame
%  tranimate   aimate coordinate frame
end

function interp_test(tc)
    
    interp1(tc, 'SO2');
    interp1(tc, 'SE2');
    interp1(tc, 'SO3');
    interp1(tc, 'SE3')
    
    function interp1(tc, cn)
        a1 = eval(sprintf('%s.rand', cn));
        a2 = eval(sprintf('%s.rand', cn));
        
        verifyEqual(tc, double(a1.interp(a2, 0)), double(a1), 'absTol',1e-10);
        verifyEqual(tc, double(a1.interp(a2, 1)), double(a2), 'absTol',1e-10);
        
        errmsg = sprintf('RTB:%s:interp:badarg', cn);
        verifyError(tc, @() a1.interp(a2, -1), errmsg );
        verifyError(tc, @() a1.interp(a2, 1.2), errmsg );
    end
end

function tests_test(tc)
    a = SO2; 
    verifyFalse(tc, isrot(a) );
    verifyTrue(tc, isrot2(a) );
    verifyFalse(tc, ishomog(a) );
    verifyFalse(tc, ishomog2(a) );
    
    verifyFalse(tc, isSE(a) );
    verifyEqual(tc, dim(a), 2);
    verifyFalse(tc, issym(a) );
    
    a = SE2; 
    verifyFalse(tc, isrot(a) );
    verifyFalse(tc, isrot2(a) );
    verifyFalse(tc, ishomog(a) );
    verifyTrue(tc, ishomog2(a) );
    
    verifyTrue(tc, isSE(a) );
    verifyEqual(tc, dim(a), 3);
    verifyFalse(tc, issym(a) );    
    
    a = SO3;
    verifyTrue(tc, isrot(a) );
    verifyFalse(tc, isrot2(a) );
    verifyFalse(tc, ishomog(a) );
    verifyFalse(tc, ishomog2(a) );    
    
    verifyFalse(tc, isSE(a) );
    verifyEqual(tc, dim(a), 3);
    verifyFalse(tc, issym(a) );
    
    a = SE3;
    verifyFalse(tc, isrot(a) );
    verifyFalse(tc, isrot2(a) );
    verifyTrue(tc, ishomog(a) );
    verifyFalse(tc, ishomog2(a) );
    
    verifyTrue(tc, isSE(a) );
    verifyEqual(tc, dim(a), 4);
    verifyFalse(tc, issym(a) );
    
    syms theta x y z
    a = SE3.Rx(theta);
    verifyTrue(tc, issym(a) );
    
    a = SE3(x, y, z);
    verifyTrue(tc, issym(a) );
    
    a = SO2(theta);
    verifyTrue(tc, issym(a) );
    
    a = SE2(x, y, theta);
    verifyTrue(tc, issym(a) );
end