## Synopsis

Robotics Toolbox for MATLAB release 10.
The toolbox contains functions and classes to represent orientation and pose in 2D and 3D (SO(2), SE(2), SO(3), SE(3)) as matrices, quaternions, twists, triple angles, and matrix exponentials. The Toolbox also provides functions for manipulating and converting between datatypes such as vectors, homogeneous transformations and unit-quaternions which are necessary to represent 3-dimensional position and orientation.

The Toolbox uses a very general method of representing the kinematics and dynamics of serial-link manipulators as MATLAB®  objects –  robot objects can be created by the user for any serial-link manipulator and a number of examples are provided for well known robots from Kinova, Universal Robotics, Rethink as well as classical robots such as the Puma 560 and the Stanford arm.

The toolbox also supports mobile robots with functions for robot motion models (unicycle, bicycle), path planning algorithms (bug, distance transform, D*, PRM), kinodynamic planning (lattice, RRT), localization (EKF, particle filter), map building (EKF) and simultaneous localization and mapping (EKF), and a Simulink model a of non-holonomic vehicle.  The Toolbox also including a detailed Simulink model for a quadrotor flying robot.

Advantages of the Toolbox are that:
  * the code is mature and provides a point of comparison for other implementations of the same algorithms;
  * the routines are generally written in a straightforward manner which allows for easy understanding, perhaps at the expense of computational efficiency. If you feel strongly about computational efficiency then you can always rewrite the function to be more efficient, compile the M-file using the Matlab compiler, or create a MEX version;
  * since source code is available there is a benefit for understanding and teaching.

## Code Example

```
>> rotx(0.2)  % SO(3) rotation matrix
ans =
    1.0000         0         0
         0    0.9801   -0.1987
         0    0.1987    0.9801
```

## Motivation

This toolbox brings robotics specific functionality to MATLAB, exploiting the native 
capabilities of MATLAB (linear algebra, portability, graphics).

## Installation

You need to have a recent version of MATLAB, R2016b or later.

Installable distributions of the toolbox are available from the [download page](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)

You can also this, my working version of the toolbox, directly:
  * clone this repo
  * also clone the [toolbox-common-matlab](https://github.com/petercorke/toolbox-common-matlab) repo
  * add both folders to your MATLAB path
  * run the demo
```
>>> rtbdemo
```

Distributions are built using the tools in the `distrib` and `doc` folders.
  
## API Reference

## Contributors

Contributions welcome.  There's a user forum at http://tiny.cc/rvcforum

## License

This toolbox is released under GNU LGPL.
