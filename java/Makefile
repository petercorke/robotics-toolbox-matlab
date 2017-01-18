.SUFFIXES: .java .class
REV = 1.7
JAVA = javac
JARFILE = DHFactor.jar

CLASSES = DHFactor.class Element.class ElementList.class


$(JARFILE): $(CLASSES)
	jar cvf $@ $^

.java.class:
	@$(JAVA) -target $(REV) -source $(REV) $<

clean:
	-rm $(CLASSES) $(JARFILE)
