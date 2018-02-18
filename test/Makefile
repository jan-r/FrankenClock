CPP=g++
CPPFLAGS=-g -Wall -I.
LDFLAGS=

all: streamprint dcf77dec

clean:
	rm -f streamprint

common: StreamReader.cpp StreamReader.h
	@echo Compiling common objects...
	$(CPP) $(CPPFLAGS) -o StreamReader.o -c StreamReader.cpp

streamprint: streamprint.cpp common
	@echo Building $@...
	$(CPP) $(CPPFLAGS) -o streamprint.o -c streamprint.cpp
	$(CPP) $(LDFLAGS) -o $@ StreamReader.o streamprint.o
	

dcf77dec: ../DCF77Decoder.cpp ../DCF77Decoder.h NoArduinoStubs.h dcf77dec.cpp
	@echo
	@echo Building $@...
	$(CPP) $(CPPFLAGS) -DNOARDUINO -o DCF77Decoder.o -c ../DCF77Decoder.cpp
	$(CPP) $(CPPFLAGS) -o dcf77dec.o -c dcf77dec.cpp
	$(CPP) $(CPPFLAGS) -o $@  DCF77Decoder.o dcf77dec.o
	

