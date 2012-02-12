# build for 32bit
MEXOPT += -maci
MEXOPT += -I/opt/local/include

OTHER = ArmControl.cpp Command.cpp SSC32Controller.cpp

build: crustcrawler.mexmaci

crustcrawler.mexmaci:  crustcrawler.cpp
	echo $(PATH)
	mex $(MEXOPT) crustcrawler.cpp $(OTHER)

clean:
	rm crustcrawler.mexmaci
