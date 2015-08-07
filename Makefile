CC = gcc
CXX = g++
AR = ar
LD = g++
CFLAGS = -Wall -fexceptions -lpthread -lm -O3 -fpermissive -fno-strict-aliasing
TARGET = fm_transmitter

all: release

OBJS = main.o wave_reader.o stdin_reader.o error_reporter.o transmitter.o

release: $(OBJS)
	$(LD) $(CFLAGS) -o $(TARGET) $(OBJS)

wave_reader.o: wave_reader.cpp wave_reader.h
	$(CC) $(CFLAGS) -c wave_reader.cpp

stdin_reader.o: stdin_reader.cpp
	$(CC) $(CFLAGS) -c stdin_reader.cpp

error_reporter.o: error_reporter.cpp
	$(CC) $(CFLAGS) -c error_reporter.cpp

transmitter.o: transmitter.cpp
	$(CC) $(CFLAGS) -c transmitter.cpp

main.o: main.cpp
	$(CC) $(CFLAGS) -c main.cpp

clean:
	rm *.o
