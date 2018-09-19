% test the Simulink RPY and Euler blocks
% in conjunction with sl_tripleangleTest.m

function tests = sl_tripleangleTest
    tests = functiontests(localfunctions);
    clc
end

function setupOnce(tc)
    sl_tripleangle
end

function teardownOnce(tc)
    bdclose
end

function radians_test(tc)
    angles = [0.3 0.4 0.5];
    seq = 'XYZ';
    set_param('sl_tripleangle/a', 'Value', num2str(angles(1)))
    set_param('sl_tripleangle/b', 'Value', num2str(angles(2)))
    set_param('sl_tripleangle/c', 'Value', num2str(angles(3)))
    set_param('sl_tripleangle/rpy2T', 'degrees' ,'off')
    set_param('sl_tripleangle/T2rpy', 'degrees' ,'off')
    set_param('sl_tripleangle/T2rpy', 'sequence', seq)
    set_param('sl_tripleangle/rpy2T', 'sequence', seq)
    set_param('sl_tripleangle/eul2T', 'degrees' ,'off')
    set_param('sl_tripleangle/T2eul', 'degrees' ,'off')
    [~,~,y] = sim('sl_tripleangle');
    tc.verifyEqual(y(1,:), [angles angles], 'AbsTol', 1e-10);
    
    seq = 'ZYX';
    set_param('sl_tripleangle/T2rpy', 'sequence', seq)
    set_param('sl_tripleangle/rpy2T', 'sequence', seq)
    [~,~,y] = sim('sl_tripleangle');
    tc.verifyEqual(y(1,:), [angles angles], 'AbsTol', 1e-10);
    
    seq = 'YXZ';
    set_param('sl_tripleangle/T2rpy', 'sequence', seq)
    set_param('sl_tripleangle/rpy2T', 'sequence', seq)
    [~,~,y] = sim('sl_tripleangle');
    tc.verifyEqual(y(1,:), [angles angles], 'AbsTol', 1e-10);
end

function degrees_test(tc)
    % now in degrees
    angles = [30 40 50];
    seq = 'XYZ';
    set_param('sl_tripleangle/a', 'Value', num2str(angles(1)))
    set_param('sl_tripleangle/b', 'Value', num2str(angles(2)))
    set_param('sl_tripleangle/c', 'Value', num2str(angles(3)))
    set_param('sl_tripleangle/rpy2T', 'degrees' ,'on')
    set_param('sl_tripleangle/T2rpy', 'degrees' ,'on')
    set_param('sl_tripleangle/T2rpy', 'sequence', seq)
    set_param('sl_tripleangle/rpy2T', 'sequence', seq)
    set_param('sl_tripleangle/eul2T', 'degrees' ,'on')
    set_param('sl_tripleangle/T2eul', 'degrees' ,'on')
    [~,~,y] = sim('sl_tripleangle');
    tc.verifyEqual(y(1,:), [angles angles], 'AbsTol', 1e-10);
    
    seq = 'ZYX';
    set_param('sl_tripleangle/T2rpy', 'sequence', seq)
    set_param('sl_tripleangle/rpy2T', 'sequence', seq)
    [~,~,y] = sim('sl_tripleangle');
    tc.verifyEqual(y(1,:), [angles angles], 'AbsTol', 1e-10);
    
    seq = 'YXZ';
    set_param('sl_tripleangle/T2rpy', 'sequence', seq)
    set_param('sl_tripleangle/rpy2T', 'sequence', seq)
    [~,~,y] = sim('sl_tripleangle');
    tc.verifyEqual(y(1,:), [angles angles], 'AbsTol', 1e-10);
end