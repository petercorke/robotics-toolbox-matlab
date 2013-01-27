%% Test all the Simulink models
function test_suite = SimulinkTest
  initTestSuite;

function braitenberg_test
    sim('sl_braitenberg');
	close all

function lanechange_test
    sim('sl_lanechange');
	close all

function pursuit_test
    sim('sl_pursuit');
	close all

function drivepoint_test
    assignin('base', 'xg', [5 5]);
    assignin('base', 'x0', [8 5 pi/2]);
    sim('sl_drivepoint');
	close all

function driveline_test
    assignin('base', 'L', [1 -2 4]);
    assignin('base', 'x0', [8 5 pi/2]);
    sim('sl_driveline');
	close all

function drivepose_test
    assignin('base', 'xg', [5 5 pi/2]);
    assignin('base', 'x0', [8 5 pi/2]);
    r = sim('sl_drivepose');
	close all

function jspace_test
    mdl_puma560
    assignin('base', 'p560', p560);
    sim('sl_jspace');
	close all

function rrmc_test
    mdl_puma560
    assignin('base', 'p560', p560);
    assignin('base', 'qn', qn);
    sim('sl_rrmc');
	close all

function rrmc2_test
    mdl_puma560
    assignin('base', 'p560', p560);
    assignin('base', 'qn', qn);
    sim('sl_rrmc2');
	close all

function ztorque_test
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_ztorque');
	close all

function vlooptest_test
    sim('vloop_test');
	close all

function vloop_test2_test
    sim('vloop_test2');
	close all

function plooptest_test
    sim('ploop_test');
	close all

function ctorque_test
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_ctorque');
	close all

function feedforward_test
    mdl_puma560
    p560 = p560.nofriction();
    assignin('base', 'p560', p560);
    sim('sl_fforward');
	close all

function flexlink_test
    mdl_twolink
    assignin('base', 'twolink', twolink);
    sim('sl_flex');
	close all

function quadcopter_test
    sim('sl_quadrotor');
	close all

function quadcopter_vs_test
    sim('sl_quadrotor_vs');
	close all
