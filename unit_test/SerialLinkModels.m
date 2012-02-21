%% This is for testing the SerialLink models in the robotics Toolbox
function test_suite = TestRobotToolboxSerialLinkModels
  initTestSuite;
%%    SerialLink models
%        DHFactor               - convert elementary transformations to DH form
function Test_DHFactor
    startup_rtb;
    s = 'Rz(q1).Rx(q2).Ty(L1).Rx(q3).Tz(L2)';
    out = DHFactor(s);
    expected_out = [];
%        mdl_Fanuc10L           - Fanuc 10L (DH, kine)
function Test_mdl_fanuc10l
    mdl_fanuc10l;
%        mdl_MotomanHP6         - Motoman HP6 (DH, kine)
function Test_mdl_motomanHP6
    mdl_motomanHP6;
%        mdl_puma560            - Puma 560 data (DH, kine, dyn)
function Test_mdl_puma560
    mdl_puma560;
%        mdl_puma560akb         - Puma 560 data (MDH, kine, dyn)
function Test_mdl_puma560akb
    mdl_puma560akb;
%        mdl_stanford           - Stanford arm data (DH, kine, dyn)
function Test_mdl_stanford
    mdl_stanford;
%        mdl_S4ABB2p8           - ABB S4 2.8 (DH, kine)
function Test_mdl_S4ABB2p8
    mdl_S4ABB2p8;
%        mdl_twolink            - simple 2-link example (DH, kine)
function Test_mdl_twolink
    mdl_twolink;
