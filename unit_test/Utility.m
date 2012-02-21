%% This is for testing the Utility functions in the robotics Toolbox
function test_suite = TestRobotToolboxUtility
  initTestSuite;
%% Utility
%    about                      - summary of object size and type
function Test_about
    about([1 2 3 4;1 2 3 4]);
%    angdiff                    - subtract 2 angles modulo 2pi
function Test_angdiff
    assertElementsAlmostEqual(angdiff(pi/2,pi), -1.5708, 'absolute',1e-4);
%    circle                     - compute/draw points on a circle
function Test_circle
    x= circle([1 2],5,'n', 5 );
    assertElementsAlmostEqual(x, [6.0000    2.5451   -3.0451   -3.0451    2.5451
                                  2.0000    6.7553    4.9389   -0.9389   -2.7553],...
                                  'absolute',1e-4);
%    colnorm                    - columnwise norm of matrix
function Test_colnorm
    x= [6.0000    2.5451   -3.0451   -3.0451    2.5451
        2.0000    6.7553    4.9389   -0.9389   -2.7553];
    cn = colnorm(x);
    assertElementsAlmostEqual(cn, [6.3246    7.2188    5.8022    3.1866    3.7509],...
                                  'absolute',1e-4);
    
%    ishomog                    - true if argument is a 4x4 matrix
function Test_ishomog
    tr = [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000];
    r = [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752];
    assertTrue(ishomog(tr),'valid');
    assertFalse(ishomog(r));
   
%    isrot                      - true if argument is a 3x3 matrix
 function Test_isrot
    tr = [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000];
    r = [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752];
    assertTrue(isrot(r),'valid');
    assertFalse(isrot(tr));
              
%    isvec                      - true if argument is a 3-vector
function Test_isvec
    vh = [1 2 3];
    vv = [1;2;3];
    s = 45;
    assertTrue(isvec(vh),3);
    assertTrue(isvec(vv));
    assertFalse(isrot(s));
    
%    maniplty                   - compute manipulability
function Test_maniplty
    mdl_puma560;
    q = [0 pi/4 -pi 1 pi/4 0];
    assertElementsAlmostEqual(p560.maniplty(q), 0.0786, 'absolute',1e-4);
    assertElementsAlmostEqual(p560.maniplty(q, 'T'), 0.1112, 'absolute',1e-4);
    assertElementsAlmostEqual(p560.maniplty(q, 'R'), 2.5936, 'absolute',1e-4);
    assertElementsAlmostEqual(p560.maniplty(q, 'asada'), 0.2733, 'absolute',1e-4);

%    numcols                    - number of columns in matrix
function Test_numcols
    a = [0 0 0; 0 0 0; 0 0 0];
    b = 2;
    assertEqual(numcols(a),3);
    assertEqual(numcols(b),1);
%    numrows                    - number of rows in matrix
function Test_numrows
    a = [0 0 0; 0 0 0; 0 0 0];
    b = 3;
    assertEqual(numrows(a),3);
    assertEqual(numrows(b),1);
%    Pgraph                     - general purpose graph class
function Test_Pgraph
    g= Pgraph();
%    Polygon                    - general purpose polygon class
function Test_Polygon
    v = [1 2 1 2;1 1 2 2];
    p = Polygon(v);
%    unit                       - unitize a vector
function Test_unit
    vh = [1 2 3];
    vv = [1;2;3];
    vo = [0 0 0];
    assertElementsAlmostEqual(unit(vh), [0.2673    0.5345    0.8018], 'absolute',1e-4);
    assertElementsAlmostEqual(unit(vv), [0.2673
                                         0.5345
                                         0.8018], 'absolute',1e-4);
    assertElementsAlmostEqual(unit(vo), [NaN   NaN   NaN], 'absolute',1e-4);
%    tb_optparse                - toolbox argument parser
function Test_tb_optprase
   
        