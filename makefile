EXECUTABLE = fm_transmitter
VERSION = 0.9.6
FLAGS = -Wall -O3 -std=c++11
TRANSMITTER = -fno-strict-aliasing -I/opt/vc/include
ifdef GPIO21
	TRANSMITTER += -DGPIO21
endif

all: fm_transmitter.o mailbox.o wave_reader.o transmitter.o
	g++ -o $(EXECUTABLE) fm_transmitter.o mailbox.o wave_reader.o transmitter.o -L/opt/vc/lib -lm -lpthread -lbcm_host

mailbox.o: mailbox.cpp mailbox.hpp
	g++ $(FLAGS) -c mailbox.cpp

wave_reader.o: wave_reader.cpp wave_reader.hpp
	g++ $(FLAGS) -c wave_reader.cpp

transmitter.o: transmitter.cpp transmitter.hpp
	g++ $(FLAGS) $(TRANSMITTER) -c transmitter.cpp

fm_transmitter.o: fm_transmitter.cpp
	g++ $(FLAGS) -DVERSION=\"$(VERSION)\" -DEXECUTABLE=\"$(EXECUTABLE)\" -c fm_transmitter.cpp

clean:
	rm *.o
