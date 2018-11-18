# build RTB
ROOT = ~/code/rvc
BUILD = tmp
MANUAL = $(BUILD)/manual/robot.pdf
SRC = $(ROOT)/robot
ROBOT = rvctools/robot
COMMON = rvctools/common
CONTRIB = rvctools/contrib
LIB = ~/lib/matlab

release:  build doco zipit

doco:	pdf markup

build:  tmp allfuncs.txt

tmp:
	# clean up
	mkdir tmp

	# recurse
	$(MAKE) -C tmp -f ../Makefile $(MFLAGS) _build

_build: 

	# make a temp directory to hold the files
	mkdir -p $(BUILD) rvctools/robot rvctools/simulink rvctools/common rvctools/contrib

	# create the Manifest
	buildcontent -r `cat ../../RELEASE` --rtb ../allfuncs.txt

	# copy all m-files and classes listed in the Manifest
	mancp --verbose Manifest $(SRC) $(ROBOT)
	mancp --verbose Manifest $(ROOT)/common $(COMMON)

	# copy any contrib files that will be included
	#cp -r $(ROOT)/contrib/paretofront rvctools/contrib
	cp -r $(ROOT)/contrib/pHRIWARE $(CONTRIB)
	cp $(LIB)/arrow3.m $(CONTRIB)


	# copy any GUIDE files
	cp ../../*.fig $(ROBOT)

	# copy Java source code and jar file
	(cd $(SRC)/java; make)
	cp -r $(SRC)/java $(ROBOT)

	# copy extra packages
	cp -r $(SRC)/+ETS* $(ROBOT)

	# copy model files
	cp -r $(SRC)/models $(ROBOT)

	# copy examples and data files
	cp -r $(SRC)/examples $(ROBOT)
	cp -r $(SRC)/data $(ROBOT)
	cp -r $(SRC)/demos $(ROBOT)
	cp -r $(SRC)/mex $(ROBOT)

	# copy RST examples
	cp -r $(SRC)/RST $(ROBOT)

	# copy symbolic examples
	cp -r $(SRC)/symbolic $(ROBOT)

	# copy Apps
	mkdir $(ROBOT)/Apps
	cp -r $(SRC)/Apps/tripleangle_App.mlapp $(ROBOT)/Apps

	# copy simulink models
	cp $(SRC)/simulink/*.slx rvctools/simulink
	cp $(SRC)/simulink/*.m rvctools/simulink
	cp $(SRC)/simulink/*.txt rvctools/simulink
	rm rvctools/simulink/makeoldversions.m
	cp -r $(SRC)/simulink/R2015a rvctools/simulink

	#cp -r $(SRC)/Octave $(ROBOT)  causes the package builder to fail

	# copy interfaces
	cp -r ../../interfaces $(ROBOT)

	# copy misc documentation files
	cp $(SRC)/RELEASE $(ROBOT)
	-cp $(SRC)/CHANGES $(ROBOT)
	cp $(SRC)/README $(ROBOT)
	cp $(SRC)/CONTRIB $(ROBOT)
	cp $(SRC)/CITATION $(ROBOT)
	cp $(SRC)/LGPL-LICENCE.txt $(ROBOT)
	cp $(SRC)/startup_rtb.m $(ROBOT)

	#cp $(ROOT)/common/*.m rvctools/common
	cp $(ROOT)/startup_rvc.m rvctools


	# remove all SVN and Simulink project files from rvctools and below
	find rvctools -name .svn -type d | xargs rm -rf
	find rvctools -name slprj -type d | xargs rm -rf

pdf:	tmp $(MANUAL)

$(MANUAL):
	# recurse
	$(MAKE) -C tmp -f ../Makefile $(MFLAGS) _pdf

_pdf:
	-echo "---------build the manual"
	-\rm -rf tmpdoc
	-mkdir tmpdoc 
	-cp -r rvctools/robot/* tmpdoc
	-cp rvctools/common/* tmpdoc

	-rm tmpdoc/HiddenFigure.m
	# add a couple of things from interfaces
	-cp rvctools/robot/interfaces/VREP/*.m tmpdoc
	-cp rvctools/robot/interfaces/Arbotix.m tmpdoc
	-cp rvctools/robot/models/*.m tmpdoc

	# copy over the manual skeleton, we will work here
	cp -r $(ROOT)/robot/doc/manual .
	
	# generate LaTeX code for all functions in there
	(cd tmpdoc; help3.py -v -l --include @* *.m; mv all.tex ../manual)

	# copy over the index generator support
	-mv funcidx* manual

	# build the PDF manual
	(cd manual; make)

	cp manual/robot.pdf rvctools/robot
	open -a pdfreduce rvctools/robot

	# and let's take a look
	open rvctools/robot/robot.pdf 


CMD = '/INSERT_HERE/ {\
r helpfuncbycat.xml\
d\
}'

markup: $(MANUAL)
	# rely on pdf code to build all the temp files
	# recurse
	$(MAKE) -C tmp -f ../Makefile $(MFLAGS) _markup

_markup:
	# build the XML/info stuff for matlab help browser system
	# build it in info, info/html
	-mkdir -p info/html
	cp -r $(SRC)/doc/info/robot_product_page.html info
	(cd tmpdoc; help3.py --rtb -m @* *.m; mv *.html ../info/html)

	# copy over the autogen files
	cp contents.html info
	cp contents_alpha.html info
	cp contents_toc.html info

# create the helptoc.xml file
	cp $(SRC)/doc/info/helptoc.xml info
	#sed '/<!--INSERT_HERE-->/r helpfuncbycat.xml' < $(SRC)/info/helptoc.xml > info/helptoc.xml

	# copy invariant files from the distro
	cp $(SRC)/doc/info/*.html info
	cp $(SRC)/doc/info/*.css info
	#cp -r $(SRC)/info/icons info
	#cp manual/figs/rtb-montage.png info

	
	# build the web/HTML doco
	-mkdir html
	(cd tmpdoc; help3.py --rtb -w @* *.m; mv *.html ../html)
	cp index.html html
	cp index_alpha.html html

	# classical matlab toolbox index
	cp Contents.m rvctools/robot

	# install it all
	cp -r info rvctools
	cp $(SRC)/doc/info.xml rvctools


zipit:
	# build the distrib archives
	(cd tmp; zip -rq ../robot-`cat ../../RELEASE`.zip ./rvctools)
	#tar zcf robot.tgz ./rvctools

test: zipit
	(cd $(ROOT); unzip $(ROOT)/robot/distrib/robot.zip)

flat: tmp pdf
	$(MAKE) -C tmp -f ../Makefile $(MFLAGS) _flat

_flat:
	-\rm -rf flat
	mkdir flat
	-cp rvctools/robot/* flat
	cp rvctools/common/*.m flat
	cp -r rvctools/robot/@SerialLink flat
	cp -r rvctools/robot/demos flat
	cp -r rvctools/robot/examples flat
	cp -r rvctools/robot/mex flat


dropbox:
	cp -r ./{robot,simulink,common} $(HOME)/Dropbox

install:
	scp -C robot-`cat ../RELEASE`.zip geek:www/RTB/r9
	#scp -C robot.zip robot.tgz geek:www/RTB/r9
	scp -Cr html geek:www/RTB/r9
	scp -C rvctools/robot/robot.pdf geek:www/RTB/r9
	ssh geek chmod -R a+rX www/RTB/r9

clean:
	@-\rm -rf tmp *.zip *.mltbx

