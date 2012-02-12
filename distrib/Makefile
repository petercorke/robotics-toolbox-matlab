ROOT = ~/lib/matlab/rvctools

release:  install doco zipit

install: 
	# clean up
	-\rm -rf simulink common tmpdoc robot robot.zip robot.tgz

	# make a temp directory to hold the files
	mkdir robot simulink common

	# create the Manifest
	buildcontent $<

	# copy all m-files and classes listed in the Manifest
	mancp Manifest .. robot
	mancp Manifest $(ROOT)/common common

	# copy Java source code and jar file
	cp -r ../*.java robot
	(cd ..; make)
	cp ../*.jar robot

	# copy simulink models
	cp -r ~/lib/matlab/rvctools/simulink robot
	cp -r ../examples robot
	cp -r ../mex robot
	cp -r ../demos robot

	# copy misc documentation files
	cp ../README robot
	cp ../RELEASE robot
	cp ../CONTRIB robot
	cp ../CITATION robot
	cp ../LGPL-LICENCE.txt robot
	cp ../startup.m robot

	cp -r $(ROOT)/simulink .
	cp $(ROOT)/common/*.m common


	# remove all SVN files
	\rm -rf `find . -name .svn`

doco:
	-\rm -rf info tmpdoc
	-mkdir tmpdoc
	-mkdir -p robot/info/html
	-cp -r robot/* tmpdoc
	-cp common/* tmpdoc

	# build the PDF manual
	(cd tmpdoc; help3.py -l --include @* *.m; mv all.tex ../../doc)
	(cd ../doc; make)
	mv ../doc/robot.pdf robot

	# build the XML/info stuff for matlab help browser system
	-mkdir -p robot/info/html
	cp -r ../info/robot_product_page.html robot/info
	(cd tmpdoc; help3.py --rtb -m @* *.m; mv *.html ../robot/info/html)
	cp contents.html robot/info
	cp contents_alpha.html robot/info
	cp contents_toc.html robot/info

	cp ../info.xml robot
	cp ../info/helptoc.xml robot/info
	cp ../info/*.html robot/info
	cp -r ../info/icons robot/info
	cp -r ../doc/rtb-montage-notext.png robot/info

	# copy over the autogen files
	cp helpfuncbycat.xml robot/info
	cp contents_toc.html robot/info

	# build the web/HTML doco
	-mkdir html
	(cd tmpdoc; help3.py --rtb -w @* *.m; mv *.html ../html)
	cp index.html html
	cp index_alpha.html html

	# classical matlab toolbox index
	cp Contents.m robot


zipit:
	# build the distrib archives
	zip -rq robot.zip ./{robot,simulink,common}
	tar zcf robot.tgz ./{robot,simulink,common}

dropbox:
	cp -r ./{robot,simulink,common} $(HOME)/Dropbox

web:
	cp -r html $(HOME)/doc/book/website/toolboxes/html/rtb

	
clean:
	@-\rm -rf robot.zip robot.tgz
	@-\rm -rf simulink common tmpdoc robot robot.zip robot.tgz
	@-\rm -rf info tmpdoc html
	@-\rm *.m *.xml *.html *.png Manifest
