rne:	frne.c ne.c vmath.c
	mex CFLAGS=-std=c99 frne.c ne.c vmath.c

install: rne
	for f in *.mex*; do cp $$f ../@SerialLink/rne.$${f##*.}; done

uninstall:
	rm ../@SerialLink/rne.mex*

clean:
	rm *.mex* *~
