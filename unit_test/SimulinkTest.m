%% Test all the Simulink models


function tests = SimulinkTest
  tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.StopTime = 0.5;
end

function teardownOnce(tc)
    delete('*.slxc')
    bdclose
end

function braitenberg_test(testCase)
    sim('sl_braitenberg', testCase.TestData.StopTime);
	bdclose all

end

function lanechange_test(testCase)
    sim('sl_lanechange', testCase.TestData.StopTime);
	bdclose all
end

function pursuit_test(testCase)
    sim('sl_pursuit', testCase.TestData.StopTime);
	bdclose all
end

function drivepoint_test(testCase)
    assignin('base', 'xg', [5 5]);
    assignin('base', 'x0', [0 0 pi/2]);
    sim('sl_drivepoint', testCase.TestData.StopTime);
	bdclose all
end

function driveline_test(testCase)
    assignin('base', 'L', [1 -2 4]);
    assignin('base', 'x0', [0 -4 pi]);
    sim('sl_driveline', testCase.TestData.StopTime);
	bdclose all
end

function drivepose_test(testCase)
    assignin('base', 'xg', [5 5 pi/2]);
    assignin('base', 'x0', [8 5 pi]);
    r = sim('sl_drivepose', testCase.TestData.StopTime);
	bdclose all
end

function jspace_test(testCase)
    mdl_puma560
    assignin('base', 'p560', p560);
    sim('sl_jspace', testCase.TestData.StopTime);
	bdclose all
end

function rrmc_test(testCase)
    mdl_puma560
    assignin('base', 'p560', p560);
    assignin('base', 'qn', qn);
    sim('sl_rrmc', testCase.TestData.StopTime);
	bdclose all
end

function rrmc2_test(testCase)
    mdl_puma560
    assignin('base', 'p560', p560);
    assignin('base', 'qn', qn);
    sim('sl_rrmc2', testCase.TestData.StopTime);
	bdclose all
end

function ztorque_test(testCase)
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_ztorque', testCase.TestData.StopTime);
	bdclose all
end

function vlooptest_test(testCase)
    sim('vloop_test', testCase.TestData.StopTime);
	bdclose all
end

function plooptest_test(testCase)
    G = 107.815;   % in case not previously set by vloop_test
    sim('ploop_test', 1);
	bdclose all
end

function ctorque_test(testCase)
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_ctorque', testCase.TestData.StopTime);
	bdclose all
end

function feedforward_test(testCase)
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_fforward', testCase.TestData.StopTime);
	bdclose all
end

function flexlink_test(testCase)
    mdl_twolink
    assignin('base', 'twolink', twolink);
    sim('sl_flex', testCase.TestData.StopTime);
	bdclose all
end

function quadcopter_test(testCase)
    sim('sl_quadrotor', testCase.TestData.StopTime);
	bdclose all
end

function quadcopter_vs_test(testCase)
    testCase.assumeTrue( exist('SphericalCamera') == 2 );

    sim('sl_quadrotor_vs', testCase.TestData.StopTime/10);
	bdclose all
end