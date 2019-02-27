# How it works

A number of toolbox functions such as `accel()`, `gravload()`, `coriolis()`, `inertia()`
rely on multiple calls to the function `rne()`.  The Simulink ROBOT block in turn
relies on `accel()`.  Since `rne()` is an M-file things can run rather slowly.

This directory contains source code for a MEX file `frne.mex` which typically runs better than an order of magnitude faster than the M-file.

The M-file `rne.m` checks:

- that MEX speedup is requested, that is the `fast` flag in the `SerialLink` robot object is true. Its status can be tested as `robot.fast` or seen by the presence of the _fastRNE_ tag displayed above the DH parameters.
- that a MEX file is present, ie. the file `frne.mex` exists

and if this is the case then it will call `frne.mex` rather than `rne_dh.m` or `rne_mdh.m`.

# Building the MEX file

The mex file compries 3 files written in vanilla C with no other library 
dependencies (apart from libm):


| File    | Purpose             |
|---------|---------------------|
| frne.c  | MEX function main, handles different call formats, unpacks the SerialArm object |
| ne.c    | The actual recursive Newton-Euler code for standard and modified DH parameters |
| vmath.c | A very simple vector and matrix (linalg) library |

These are compiled and linked together, resulting in a MEX file called
frne.  The extension of the file depends on the operating system and architecture.

## For Linux or MacOS just run:
```
% make
```

which assumes that the `mex` utility is in your current path.  This is typically
found in the `bin` subdirectory of your installed MATLAB.  You do of course also
need a C compiler.

## For Windows:

Probably easier to run the script from inside MATLAB
```
>> make
```
you will need to have a C compiler installed and configured within MATLAB.

MathWorks provide a free C compiler for Windows called [`MinGW`](http://www.mingw.org) (minimalist GNU compiler).  You can install this through the AddOns manager from MATLAB desktop.


# Notes on performance

The MEX file speedup is greater when invoked on multiple points, not just one.  This
is because of startup overhead in extracting the kinematic and dynamic parameters 
from the MATLAB `SerialArm` object.

## Limitations

- The MEX file does not support base or tool transforms.  `rne.m` performs a hack with the gravity vector simulate the effect of a base transform.

## Testing

The script `check.m` compares the results and execution time for two different robots (Puma560, Stanford arm which has a prismatic link) each represented using standard or modified DH parameters.  The script output is below (for an i7 Macbook Pro)

```
>> check
***************************************************************
************************ Puma 560 *****************************
***************************************************************
************************ normal case *****************************
DH:  Speedup is         25, worst case error is 0.000000
MDH: Speedup is         34, worst case error is 0.000000
************************ no gravity *****************************
DH:  Speedup is         30, worst case error is 0.000000
MDH: Speedup is         24, worst case error is 0.000000
************************ ext force *****************************
DH:  Speedup is         16, worst case error is 0.000000
MDH: Speedup is         30, worst case error is 0.000000

***************************************************************
********************** Stanford arm ***************************
***************************************************************
************************ normal case *****************************
DH:  Speedup is         35, worst case error is 0.000000
MDH: Speedup is         30, worst case error is 0.000000
************************ no gravity *****************************
DH:  Speedup is         32, worst case error is 0.000000
MDH: Speedup is         31, worst case error is 0.000000
************************ ext force *****************************
DH:  Speedup is         31, worst case error is 0.000000
MDH: Speedup is         29, worst case error is 0.000000
```


