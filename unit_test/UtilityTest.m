%% This is for testing the Utility functions in the robotics Toolbox
function test_suite = UtilityTest
  initTestSuite;
%% Utility
%    about                      - summary of object size and type
function ab_testout
    a = [1 2 3; 4 5 6];
    about(a);
    about a;

%    angdiff                    - subtract 2 angles modulo 2pi
function an_testgdiff
    assertElementsAlmostEqual(angdiff(pi/2,pi), -1.5708, 'absolute',1e-4);
%    circle                     - compute/draw points on a circle
function ci_testrcle
    x= circle([1 2],5,'n', 5 );
    assertElementsAlmostEqual(x, [6.0000    2.5451   -3.0451   -3.0451    2.5451
                                  2.0000    6.7553    4.9389   -0.9389   -2.7553],...
                                  'absolute',1e-4);
%    colnorm                    - columnwise norm of matrix
function co_testlnorm
    x= [6.0000    2.5451   -3.0451   -3.0451    2.5451
        2.0000    6.7553    4.9389   -0.9389   -2.7553];
    cn = colnorm(x);
    assertElementsAlmostEqual(cn, [6.3246    7.2188    5.8022    3.1866    3.7509],...
                                  'absolute',1e-4);
    
%    ishomog                    - true if argument is a 4x4 matrix
function is_testhomog
    tr = [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000];
    r = [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752];
    assertTrue(ishomog(tr));
    assertTrue(ishomog(cat(3, tr, tr)));
    assertTrue(ishomog(tr),'valid');
    assertFalse(ishomog(r));
    assertFalse(ishomog(1));
    assertFalse(ishomog(ones(4,4),'valid'));
   
%    isrot                      - true if argument is a 3x3 matrix
 function is_testrot
    tr = [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000];
    r = [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752];
    assertTrue(isrot(r));
    assertTrue(isrot(cat(3, r,r)));
    assertTrue(isrot(r),'valid');
    assertFalse(isrot(tr));
    assertFalse(isrot(1));
    assertFalse(isrot(ones(3,3),'valid'));
              
%    isvec                      - true if argument is a 3-vector
function is_testvec
    vh = [1 2 3];
    vv = [1;2;3];
    s = 45;
    assertTrue(isvec(vh),3);
    assertTrue(isvec(vv));
    assertFalse(isvec(s));
    assertFalse(isvec(ones(2,2)));
    assertFalse(isvec(ones(2,2,2)));
    
%    numcols                    - number of columns in matrix
function nu_testmcols
    a = ones(2,3,4);
    b = 2;
    assertEqual(numcols(a),3);
    assertEqual(numcols(b),1);
%    numrows                    - number of rows in matrix
function nu_testmrows
    a = ones(2,3,4);
    b = 3;
    assertEqual(numrows(a),2);
    assertEqual(numrows(b),1);

%    Polygon                    - general purpose polygon class
function Po_testlygon
    v = [1 2 1 2;1 1 2 2];
    p = Polygon(v);
%    unit                       - unitize a vector
function un_testit
    vh = [1 2 3];
    vv = [1;2;3];
    vo = [0 0 0];
    assertElementsAlmostEqual(unit(vh), [0.2673    0.5345    0.8018], 'absolute',1e-4);
    assertElementsAlmostEqual(unit(vv), [0.2673
                                         0.5345
                                         0.8018], 'absolute',1e-4);

    assertExceptionThrown( @() unit(vo), 'RTB:unit:zero_norm');

%    tb_optparse                - toolbox argument parser
function tb_test_optparse

    opt.one = [];
    opt.two = 2;
    opt.three = 'three';
    opt.four = false;
    opt.five = true;
    opt.color = {'red', 'green', 'blue'};
    opt.select = {'#bob', '#nancy'};

    opt2 = tb_optparse(opt, {'verbose'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'one', 7});
    assertEqual(opt2.one, 7);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    assertExceptionThrown(@() tb_optparse(opt, {'one'}), 'RTB:tboptparse:badargs');

    opt2 = tb_optparse(opt, {'two', 3});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 3);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'three', 'bob'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'bob');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'four'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, true);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'nofour'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'nofive'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, false);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'green'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'green');
    assertEqual(opt2.select, 1);

    opt2 = tb_optparse(opt, {'nancy'});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, true);
    assertEqual(opt2.color, 'red');
    assertEqual(opt2.select, 2);

    opt2 = tb_optparse(opt, {});
    assertEqual(opt2.verbose, false);
    assertEqual(opt2.debug, 0);
    opt2 = tb_optparse(opt, {'verbose'});
    assertEqual(opt2.verbose, true);
    opt2 = tb_optparse(opt, {'verbose=2'});
    assertEqual(opt2.verbose, 2);
    opt2 = tb_optparse(opt, {'debug', 11});
    assertEqual(opt2.debug, 11);

    opt2 = tb_optparse(opt, {'showopt'});

    opt3.color = 'green';
    opt3.five = false;

    opt2 = tb_optparse(opt, {'setopt', opt3});
    assertEqual(opt2.one, []);
    assertEqual(opt2.two, 2);
    assertEqual(opt2.three, 'three');
    assertEqual(opt2.four, false);
    assertEqual(opt2.five, false);
    assertEqual(opt2.color, 'green');
    assertEqual(opt2.select, 1);

    [opt2,args] = tb_optparse(opt, {1, 'three', 4, 'spam', 2, 'red',  'spam'});
    assertEqual(args, {1, 'spam', 2, 'spam'});

    assertExceptionThrown( @() tb_optparse(opt, {'two'}), 'RTB:tboptparse:badargs');
    assertExceptionThrown( @() tb_optparse(opt, 'bob'), 'RTB:tboptparse:badargs');

function trprint_test

    a = transl([1,2,3]) * eul2tr([.1, .2, .3]);

    trprint(a);
    trprint(a, 'euler');
    trprint(a, 'euler', 'radian');
    trprint(a, 'rpy');
    trprint(a, 'rpy', 'radian');
    trprint(a, 'angvec');
    trprint(a, 'angvec', 'radian');
    trprint(a, 'angvec', 'radian', 'fmt', '%g');
    trprint(a, 'angvec', 'radian', 'fmt', '%g', 'label', 'bob');

