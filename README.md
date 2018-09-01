## Synopsis

Robotics Toolbox for MATLAB&reg; release 10.

This toolbox brings robotics specific functionality to MATLAB, exploiting the native capabilities of MATLAB (linear algebra, portability, graphics).

The toolbox contains functions and classes to represent orientation and pose in 2D and 3D (SO(2), SE(2), SO(3), SE(3)) as matrices, quaternions, twists, triple angles, and matrix exponentials. The Toolbox also provides functions for manipulating and converting between datatypes such as vectors, homogeneous transformations and unit-quaternions which are necessary to represent 3-dimensional position and orientation.

The Toolbox uses a very general method of representing the kinematics and dynamics of serial-link manipulators as MATLAB®  objects –  robot objects can be created by the user for any serial-link manipulator and a number of examples are provided for well known robots from Kinova, Universal Robotics, Rethink as well as classical robots such as the Puma 560 and the Stanford arm.

The toolbox also supports mobile robots with functions for robot motion models (unicycle, bicycle), path planning algorithms (bug, distance transform, D*, PRM), kinodynamic planning (lattice, RRT), localization (EKF, particle filter), map building (EKF) and simultaneous localization and mapping (EKF), and a Simulink model a of non-holonomic vehicle.  The Toolbox also including a detailed Simulink model for a quadrotor flying robot.

Advantages of the Toolbox are that:
  * the code is mature and provides a point of comparison for other implementations of the same algorithms;
  * the routines are generally written in a straightforward manner which allows for easy understanding, perhaps at the expense of computational efficiency. If you feel strongly about computational efficiency then you can always rewrite the function to be more efficient, compile the M-file using the Matlab compiler, or create a MEX version;
  * since source code is available there is a benefit for understanding and teaching.

## Code Example

```matlab
>> rotx(0.2)  % SO(3) rotation matrix
ans =
    1.0000         0         0
         0    0.9801   -0.1987
         0    0.1987    0.9801
```


## Installation from github

You need to have a recent version of MATLAB, R2016b or later.

The Robotics Toolbox for MATLAB has dependency on the repository `toolbox-common-matlab`.  

To install the Toolbox on your computer from github follow these simple instructions.

From the shell:

```shell
% mkdir rvctools
% cd rvctools
% git clone https://github.com/petercorke/robotics-toolbox-matlab.git robot
% git clone https://github.com/petercorke/toolbox-common-matlab.git common
% mv common/startup_rvc.m .
```

From within MATLAB
```matlab
>> cd rvctools  % this is the same folder as above
>> startup_rvc
```
The second line sets up the MATLAB path appropriately but it's only for the current session.  You can either:
1. Repeat this everytime you start MATLAB
2. Add it to your `startup.m` file
3. Once you have run startup_rvc, run `pathtool` and push the `Save` button


## Online resources:

* [Home page](http://www.petercorke.com)
* [Discussion group](http://groups.google.com/group/robotics-tool-box?hl=en)

Please email bug reports, comments or code contribtions to me at rvc@petercorke.com
  

## Contributors

Contributions welcome.  There's a user forum at http://tiny.cc/rvcforum

## License

This toolbox is released under GNU LGPL.
