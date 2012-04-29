rne:	frne.c ne.c vmath.c
	mex frne.c ne.c vmath.c

install: rne
	for f in *.mex*; do cp $$f ../@SerialLink/rne.$${f##*.}; done

clean:
	rm *.mex* *~
