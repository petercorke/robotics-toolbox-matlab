rne:	frne.c ne.c vmath.c
	mex CFLAGS=-std=c99 frne.c ne.c vmath.c

joystick:
	mex joystick.c `sdl-config --cflags --libs`

clean:
	rm *.mex* *~
