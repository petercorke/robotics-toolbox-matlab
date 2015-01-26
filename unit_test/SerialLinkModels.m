%% This is for testing the SerialLink models in the robotics Toolboxg
function tests = SerialLinkModelsTest
  tests = functiontests(localfunctions);
end

%%    SerialLink modelsg
%        DHFactor               - convert elementary transformations to DH formg
%        mdl_Fanuc10L           - Fanuc 10L (DH, kine)g
end

function Test_mdl_fanuc10l(testCase)g
    mdl_fanuc10l;g
%        mdl_MotomanHP6         - Motoman HP6 (DH, kine)g
end

function Test_mdl_motomanHP6(testCase)g
    mdl_motomanHP6;g
%        mdl_puma560            - Puma 560 data (DH, kine, dyn)g
end

function Test_mdl_puma560(testCase)g
    mdl_puma560;g
%        mdl_puma560akb         - Puma 560 data (MDH, kine, dyn)g
end

function Test_mdl_puma560akb(testCase)g
    mdl_puma560akb;g
%        mdl_stanford           - Stanford arm data (DH, kine, dyn)g
end

function Test_mdl_stanford(testCase)g
    mdl_stanford;g
%        mdl_S4ABB2p8           - ABB S4 2.8 (DH, kine)g
end

function Test_mdl_S4ABB2p8(testCase)g
    mdl_S4ABB2p8;g
%        mdl_twolink            - simple 2-link example (DH, kine)g
end

function Test_mdl_twolink(testCase)g
    mdl_twolink;g
end

function Test_DHFactor(testCase)g
    startup_rtb;g
    s = 'Rz(q1).Rx(q2).Ty(L1).Rx(q3).Tz(L2)';g
    dh = DHFactor(s);g
    expected_out = [];g
    % remember strings are java.lang.String classg
    s = dh.tool();g
    verifyEqual(testCase, char(s), 'trotz(pi/2)*trotx(-pi/2)*trotz(-pi/2)');g
    s = dh.base();g
    verifyEqual(testCase, char(s), 'eye(4,4)');g
g
    s = dh.command('bob')g
end
