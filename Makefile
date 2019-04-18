install:
	@echo "========================================= build mex files"
	make -C mex

	@echo "========================================= build java classes"
	make -C java
