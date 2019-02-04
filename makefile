EXECUTABLE = fm_transmitter
VERSION = 0.9.1
FLAGS = -Wall -O3

all: main.o mailbox.o error_reporter.o sample.o preemp.o wave_reader.o transmitter.o
	g++ -L/opt/vc/lib -lm -lpthread -lbcm_host -o $(EXECUTABLE) main.o mailbox.o sample.o preemp.o error_reporter.o wave_reader.o transmitter.o

mailbox.o: mailbox.c mailbox.h
	g++ $(FLAGS) -c mailbox.c
	
error_reporter.o: error_reporter.cpp error_reporter.h
	g++ $(FLAGS) -c error_reporter.cpp

sample.o: sample.cpp sample.h
	g++ $(FLAGS) -c sample.cpp

preemp.o: preemp.cpp preemp.h
	g++ $(FLAGS) -c preemp.cpp
	
wave_reader.o: wave_reader.cpp wave_reader.h
	g++ $(FLAGS) -c wave_reader.cpp

transmitter.o: transmitter.cpp transmitter.h
	g++ $(FLAGS) -fno-strict-aliasing -I/opt/vc/include -c transmitter.cpp

main.o: main.cpp
	g++ $(FLAGS) -DVERSION=\"$(VERSION)\" -DEXECUTABLE=\"$(EXECUTABLE)\" -c main.cpp

clean:
	rm *.o
