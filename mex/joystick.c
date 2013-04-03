/*
 * joystick.c
 *
 * Use SDL 1.2.15 to read gamepad joystick and button values in
 * a portable fashion.
 *
 * Usage:
 *
 * % return vector of joystick values in range -1 to +1
 * >> j = joystick
 * % also return vector of button values, either 0 (not pressed) or 1 (pressed)
 * >> [j,b] = joystick
 *
 * To build::
 * Have SDL installed: apt-get install sdl, brew install sdl
 *
 * mex joystick.c `sdl-config --cflags --libs`
 * 
 * Copyright (c) 2013 Peter Corke
 */

// SDL defines
#include    "SDL.h"
#include    "SDL_joystick.h"
#include    "SDL_video.h"

// Matlab defines
#include    "mex.h"

#include    <stdio.h>
#include    <math.h>

// Output Arguments
#define	JOY_OUT	    plhs[0]
#define	BUTTON_OUT	plhs[1]

// Persistent data between calls
SDL_Joystick *joy;   // SDL joystick object pointer
double  *joydata;    // joystick data
int     njoy;        // number of joystick axes
double  *buttondata; // button data
int     nbutton;     // number of buttons

static void
cleanup(void)
{
    // free allocated arrays
    if (joydata)
        free(joydata);
    if (buttondata)
        free(buttondata);
}

void
mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

	// Check for proper number of arguments
    if (nrhs != 0) {
		mexErrMsgTxt("JOYSTICK requires zero input arguments.");
	}

    if (joy == NULL) {
        // init the joystick if none initialized

        // Gotch: must initialize video as well!
        //  see comments in sdl/test/testjoystick
        if ( SDL_Init( SDL_INIT_VIDEO| SDL_INIT_JOYSTICK  ) != 0 ) {
            mexErrMsgTxt("Couldnt initialize SDL");
        }

        // Check for a joystick
        if (SDL_NumJoysticks() > 0) {
          // Open joystick
          SDL_JoystickEventState(SDL_ENABLE);
          joy = SDL_JoystickOpen(0);
          
          if(joy) {
            printf("Opened Joystick 0\n");
            printf("Name: %s\n", SDL_JoystickName(0));
            printf("Number of Axes: %d\n", SDL_JoystickNumAxes(joy));
            printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(joy));
            printf("Number of Balls: %d\n", SDL_JoystickNumBalls(joy));
            printf("Number of Hats: %d\n", SDL_JoystickNumHats(joy));

            // record the number of axes and buttons
            njoy = SDL_JoystickNumAxes(joy);
            nbutton = SDL_JoystickNumButtons(joy);

            // allocate storage
            joydata = (double *)calloc(njoy, sizeof(double));
            buttondata = (double *)calloc(nbutton, sizeof(double));

            mexAtExit(cleanup);
          }
          else
            mexErrMsgTxt("Couldn't open Joystick 0\n");
        }
    }

    // process all events in the queue and update global data
    SDL_Event event;
    while (SDL_PollEvent(&event) ) {

        switch (event.type) {
        case SDL_JOYAXISMOTION:
            joydata[event.jaxis.axis] = event.jaxis.value / 32768.0;
            break;
        case SDL_JOYBUTTONDOWN:
            buttondata[event.jbutton.button] = 1;
            break;
        case SDL_JOYBUTTONUP:
            buttondata[event.jbutton.button] = 0;
            break;
        case SDL_QUIT:
            break;
        }
    }

    // create return values

    if (nlhs > 0) {
        int i;
        // copy out the joystick data
        JOY_OUT = mxCreateDoubleMatrix(1, njoy, mxREAL);
        double *p = mxGetPr(JOY_OUT);
        for (i=0; i<njoy; i++)
            *p++ = joydata[i];
    }
    if (nlhs > 1) {
        int i;
        // copy out the button data
        BUTTON_OUT = mxCreateDoubleMatrix(1, nbutton, mxREAL);
        double *p = mxGetPr(BUTTON_OUT);
        for (i=0; i<nbutton; i++)
            *p++ = buttondata[i];
    }
}
