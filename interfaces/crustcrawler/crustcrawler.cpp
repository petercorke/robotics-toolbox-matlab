/*=================================================================
 * crustcrawler.cpp 
 * 
 * This example is the C++ version of mexatexit.c.  It demonstrates
 * how to write strings to a data file using a C++ static class
 * contructor.  In this example, we do not need to use a mexatexit
 * function to close the data file. This is because in C++ when you
 * instantiate a static class constructor, the destructor for that
 * static class gets called automatically when the MEX-file is cleared
 * or exited.  
 *
 * The input to the MEX-file is a string.  You may continue calling
 * the function with new strings to add to the data file
 * matlab.data. The data file will not be closed until the MEX-file is
 * cleared or MATLAB is exited.

 * This is a MEX-file for MATLAB.  
 * Copyright 1984-2006 The MathWorks, Inc.
 * All rights reserved.
 *=================================================================*/

/* $Revision: 1.2.6.2 $ */

#include <stdio.h>
#include <string.h> /* strlen */
#include "mex.h"

// CTY arm project
#include "ArmControl.hpp"


// Boost
#include <boost/numeric/ublas/vector.hpp>

using namespace boost::numeric;

SSC32Controller *ssc;
ArmControl *ctl;

/*
 * crustcrawler('init', serialport, baudrate)   open the link
 * crustcrawler('stop')   stop motion
 * crustcrawler('park')   park the robot
 * crustcrawler('move', q, time)   move to joint angles
 * q = crustcrawler('get')   get joint angles
 */

void 
mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    char *str;
    
    /* Check for proper number of input and output arguments */    
    if (nrhs == 0) {
        mexPrintf("Open connection to robot");
        
        //SSC32Controller ssc(port);
        ssc = new SSC32Controller("/dev/tty.usbserial-00001104", SSC32Controller::B_9600);
        ctl = new ArmControl(*ssc);

        ctl->setRateLimit(500);

        ublas::vector<double> angle_limits(ublas::vector<double>(NUM_JOINTS));

        // max limits
        angle_limits(0) = 3.0/8.0 * M_PI;
        angle_limits(1) = M_PI_2;
        angle_limits(2) = M_PI - .70; // off arm brace
        angle_limits(3) = M_PI_2;
        mexPrintf("set max limits\n");
        ctl->setMaxAngle(angle_limits);

        // min limits
        angle_limits(0) = -3.0/8.0 * M_PI;
        angle_limits(1) = -M_PI_2 + 0.35; // off spring pedestal
        //  angle_limits(2) =  0;
        angle_limits(2) =  -50.0*2.0*M_PI/360.0;
        angle_limits(3) = -M_PI_2;
        ctl->setMinAngle(angle_limits);
        mexPrintf("set min limits\n");


    } else if (nrhs > 0) {
        if (!(mxIsChar(prhs[0]))) {
            mexErrMsgTxt("Input must be of type string.\n.");
        }
        str=mxArrayToString(prhs[0]);
        if (strcmp(str, "stop") == 0) {
            mexPrintf("STOP\n");
            ctl->stop();
        } else if (strcmp(str, "park") == 0) {
            mexPrintf("PARK\n");
            ctl->park();
        } else if (strcmp(str, "home") == 0) {
            mexPrintf("HOME\n");
            ctl->home();
        } else if (strcmp(str, "move") == 0) {
            if (nrhs != 3)
                mexErrMsgTxt("Three arguments for move command\n.");

            ublas::vector<double> new_position(NUM_JOINTS);

            if (mxGetNumberOfElements(prhs[1]) != 4)
                mexErrMsgTxt("Joint vector must have 4 elements\n.");
                
            double *q = mxGetPr(prhs[1]);
            double time = mxGetScalar(prhs[2]);
            
            for (int i = 0; i < 4; i++ )
                new_position(i) = q[i];
            ctl->moveToPosition(new_position, (int) time);
        } else if (strcmp(str, "gripper") == 0) {
            if (nrhs != 2)
                mexErrMsgTxt("Two arguments for gripper command\n.");
            if (mxGetScalar(prhs[1]) == 0)
                ctl->grabMarker();
            else
                ctl->openGrip();
        } else if (strcmp(str, "get") == 0) {
            if (nlhs < 1)
                mexErrMsgTxt("No output argument for return value\n.");

            mxArray *m = mxCreateDoubleMatrix(5, 1, mxREAL);

            double *q = mxGetPr(m);
            for (int i = 0; i < 4; i++ )
                q[i] = ctl->position(i);
            plhs[0] = m;
        }
        mxFree(str);
    }
    if (nlhs > 1){
        mexErrMsgTxt("Too many output arguments.");
    }
    
    return;
}
