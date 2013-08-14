all:    rne joystick

rne:	frne.c ne.c vmath.c
	mex CFLAGS=-std=c99 frne.c ne.c vmath.c

joystick:
	mex joystick.c `sdl-config --cflags --libs`

install: rne
	for f in frne.mex*; do cp $$f ../@SerialLink; done

uninstall:
	rm ../@SerialLink/frne.mex*

clean:
	rm *.mex* *~
