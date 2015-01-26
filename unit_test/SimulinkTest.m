%% Test all the Simulink models
end
function test_suite(testCase) = SimulinkTest
  initTestSuite;

end
function braitenberg_test(testCase)
    sim('sl_braitenberg');
	close all

end
function lanechange_test(testCase)
    sim('sl_lanechange');
	close all

end
function pursuit_test(testCase)
    sim('sl_pursuit');
	close all

end
function drivepoint_test(testCase)
    assignin('base', 'xg', [5 5]);
    assignin('base', 'x0', [8 5 pi/2]);
    sim('sl_drivepoint');
	close all

end
function driveline_test(testCase)
    assignin('base', 'L', [1 -2 4]);
    assignin('base', 'x0', [8 5 pi/2]);
    sim('sl_driveline');
	close all

end
function drivepose_test(testCase)
    assignin('base', 'xg', [5 5 pi/2]);
    assignin('base', 'x0', [8 5 pi/2]);
    r = sim('sl_drivepose');
	close all

end
function jspace_test(testCase)
    mdl_puma560
    assignin('base', 'p560', p560);
    sim('sl_jspace');
	close all

end
function rrmc_test(testCase)
    mdl_puma560
    assignin('base', 'p560', p560);
    assignin('base', 'qn', qn);
    sim('sl_rrmc');
	close all

end
function rrmc2_test(testCase)
    mdl_puma560
    assignin('base', 'p560', p560);
    assignin('base', 'qn', qn);
    sim('sl_rrmc2');
	close all

end
function ztorque_test(testCase)
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_ztorque');
	close all

end
function vlooptest_test(testCase)
    sim('vloop_test');
	close all

end
function vloop_test2_test(testCase)
    sim('vloop_test2');
	close all

end
function plooptest_test(testCase)
    sim('ploop_test');
	close all

end
function ctorque_test(testCase)
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_ctorque');
	close all

end
function feedforward_test(testCase)
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_fforward');
	close all

end
function flexlink_test(testCase)
    mdl_twolink
    assignin('base', 'twolink', twolink);
    sim('sl_flex');
	close all

end
function quadcopter_test(testCase)
    sim('sl_quadrotor');
	close all

end
function quadcopter_vs_test(testCase)
    sim('sl_quadrotor_vs');
	close all
