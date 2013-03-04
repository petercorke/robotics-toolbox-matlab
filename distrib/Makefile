ROOT = ~/lib/matlab/rvctools

release:  build doco zipit

build: 
	# clean up
	-\rm -rf rvctools tmpdoc robot.zip robot.tgz

	# make a temp directory to hold the files
	mkdir -p rvctools/robot rvctools/simulink rvctools/common

	# create the Manifest
	buildcontent -r `cat ../RELEASE` --rtb $<

	# copy all m-files and classes listed in the Manifest
	mancp --verbose Manifest .. rvctools/robot
	mancp --verbose Manifest $(ROOT)/common rvctools/common

	# copy Java source code and jar file
	cp -r ../*.java rvctools/robot
	(cd ..; make)
	cp ../*.jar rvctools/robot

	# copy simulink models
	cp -r ../simulink/*.{mdl,m} rvctools/simulink
	cp -r ../examples rvctools/robot
	cp -r ../demos rvctools/robot
	cp -r ../mex rvctools/robot
	cp -r ../Octave rvctools/robot

	# copy misc documentation files
	cp ../RELEASE rvctools/robot
	-cp ../CHANGES rvctools/robot
	cp ../README rvctools/robot
	cp ../CONTRIB rvctools/robot
	cp ../CITATION rvctools/robot
	cp ../LGPL-LICENCE.txt rvctools/robot
	cp ../startup_rtb.m rvctools/robot

	#cp $(ROOT)/common/*.m rvctools/common
	cp $(ROOT)/startup_rvc.m rvctools


	# remove all SVN files from rvctools and below
	find rvctools -name .svn -type d | xargs rm -rf

doco:
	-\rm -rf tmpdoc
	-mkdir tmpdoc
	-mkdir -p rvctools/robot/info/html
	-cp -r rvctools/robot/* tmpdoc
	-cp rvctools/common/* tmpdoc

	-rm tmpdoc/HiddenFigure.m

	# build the PDF manual
	(cd tmpdoc; help3.py -l --include @* *.m; mv all.tex ../../doc)
	(cd ../doc; make)
	mv ../doc/robot.pdf rvctools/robot

	# build the XML/info stuff for matlab help browser system
	-mkdir -p rvctools/robot/info/html
	cp -r ../info/robot_product_page.html rvctools/robot/info
	(cd tmpdoc; help3.py --rtb -m @* *.m; mv *.html ../rvctools/robot/info/html)
	cp contents.html rvctools/robot/info
	cp contents_alpha.html rvctools/robot/info
	cp contents_toc.html rvctools/robot/info

	cp ../info.xml rvctools/robot
	cp ../info/helptoc.xml rvctools/robot/info
	cp ../info/*.html rvctools/robot/info
	cp -r ../info/icons rvctools/robot/info
	cp -r ../doc/rtb-montage-notext.png rvctools/robot/info

	# copy over the autogen files
	cp helpfuncbycat.xml rvctools/robot/info
	cp contents_toc.html rvctools/robot/info

	# build the web/HTML doco
	-mkdir html
	(cd tmpdoc; help3.py --rtb -w @* *.m; mv *.html ../html)
	cp index.html html
	cp index_alpha.html html

	# classical matlab toolbox index
	cp Contents.m rvctools/robot


zipit:
	# build the distrib archives
	zip -rq robot-`cat ../RELEASE`.zip ./rvctools
	#tar zcf robot.tgz ./rvctools

test: zipit
	(cd $(ROOT); unzip $(ROOT)/robot/distrib/robot.zip)

dropbox:
	cp -r ./{robot,simulink,common} $(HOME)/Dropbox

install:
	scp -C robot-`cat ../RELEASE`.zip geek:www/RTB/r9
	#scp -C robot.zip robot.tgz geek:www/RTB/r9
	scp -Cr html geek:www/RTB/r9
	scp -C rvctools/robot/robot.pdf geek:www/RTB/r9
	ssh geek chmod -R a+rX www/RTB/r9

clean:
	@-\rm -rf robot.zip robot.tgz
	@-\rm -rf simulink common tmpdoc robot robot.zip robot.tgz
	@-\rm -rf info tmpdoc html
	@-\rm *.m *.xml *.html *.png Manifest
	-\rm -rf rvctools tmpdoc robot.zip robot.tgz
