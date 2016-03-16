CFLAGS += -Wall -fexceptions -pthread -lm -O3 -fpermissive -fno-strict-aliasing
TARGET = fm_transmitter

CPP=$(CCPREFIX)g++

all: main.o error_reporter.o wave_reader.o stdin_reader.o transmitter.o
	$(CPP) $(CFLAGS) -o $(TARGET) main.o error_reporter.o wave_reader.o stdin_reader.o transmitter.o

wave_reader.o: wave_reader.cpp wave_reader.h
	$(CPP) $(CFLAGS) -c wave_reader.cpp

stdin_reader.o: stdin_reader.cpp stdin_reader.h
	$(CPP) $(CFLAGS) -c stdin_reader.cpp

error_reporter.o: error_reporter.cpp error_reporter.h
	$(CPP) $(CFLAGS) -c error_reporter.cpp

transmitter.o: transmitter.cpp transmitter.h
	$(CPP) $(CFLAGS) -c transmitter.cpp

main.o: main.cpp
	$(CPP) $(CFLAGS) -c main.cpp

clean:
	rm *.o
