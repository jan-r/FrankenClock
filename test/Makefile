CPP=g++
CPPFLAGS=-g -Wall
LDFLAGS=

all: streamprint

clean:
	rm -f streamprint

common: StreamReader.cpp StreamReader.h
	@echo Compiling common objects...
	$(CPP) $(CPPFLAGS) -o StreamReader.o -c StreamReader.cpp

streamprint: streamprint.cpp common
	@echo Building streamprint...
	$(CPP) $(CPPFLAGS) -o streamprint.o -c streamprint.cpp
	$(CPP) $(LDFLAGS) -o streamprint StreamReader.o streamprint.o
	
	

