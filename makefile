CFLAGS = -Wall -fexceptions -lpthread -lm -O3 -fpermissive -fno-strict-aliasing
TARGET = fm_transmitter

all: main.o error_reporter.o wave_reader.o stdin_reader.o transmitter.o
	g++ $(CFLAGS) -o $(TARGET) main.o error_reporter.o wave_reader.o stdin_reader.o transmitter.o

wave_reader.o: wave_reader.cpp wave_reader.h
	g++ $(CFLAGS) -c wave_reader.cpp

stdin_reader.o: stdin_reader.cpp stdin_reader.h
	g++ $(CFLAGS) -c stdin_reader.cpp

error_reporter.o: error_reporter.cpp error_reporter.h
	g++ $(CFLAGS) -c error_reporter.cpp

transmitter.o: transmitter.cpp transmitter.h
	g++ $(CFLAGS) -c transmitter.cpp

main.o: main.cpp
	g++ $(CFLAGS) -c main.cpp

clean:
	rm *.o
