DH.jar: DHFactor.java
	javac DHFactor.java
	jar cf DH.jar *.class
	-\rm *.class
