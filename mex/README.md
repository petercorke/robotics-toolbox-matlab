A number of toolbox functions such as accel(), gravload(), coriolis(), inertia()
rely on multiple calls to the function rne().  The Simulink ROBOT block in turn
relies on accel().  Since rne() is an M-file things can run rather slowly.

This directory contains a drop in replacment for rne.m which is a MEX file and
typically runs 10-20 times faster.

BUILDING THE MEX FILE

The mex file compries 3 files written in vanilla C with no other library 
dependencies (apart from libm):

    frne.c
    ne.c
    vmath.c

that must be compiled and linked together, resulting in a MEX file called
frne.  The extension of the file depends on the operating system and architecture.

For Linux or MacOS just run:

% make

which assumes that the mex utility is in your current path.  This is typically
found in the bin subdirectory of your installed MATLAB.  You do of course also
need a C compiler.

For Windows:

probably easier to run the script from inside MATLAB

>> make

INSTALLING THE MEX FILE

1. Move the MEX file frne.xxx to the directory above.  You can then invoke either
rne() or frne() and evaluate the speed improvement.  The results should be exactly
the same.

2. Move the MEX file frne.xxx to the @SerialLink directory, rename it rne.xxx, and
type "clear classes".  The MEX file will now be used instead of the M-file, and thus 
anything that calls rne() will use the MEX file and be faster.  This applies to
inertia(), coriolis(), gravload(), and fdyn().

You can use the command "make install" to do this.

NOTES ON PERFORMANCE

The MEX file speedup is greater when invoked on multiple points, not just one.  This
is because of startup overhead in extracting the kinematic and dynamic parameters 
from the SerialArm object.

LIMITATIONS

- The MEX file does not support modified DH parameters.
- The MEX file does not support base or tool transforms.

NOTES ON FILES

    frne.c   MEX function main, handles different call formats, unpacks the SerialArm
             object.
    ne.c     The actual recursive Newton-Euler code for standard and modified DH 
             parameters.
    vmath.c  A very simple vector and matrix (linalg) library.
