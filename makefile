FLAGS = -Wall -fexceptions -pthread -O3 -fpermissive -fno-strict-aliasing
INCLUDES = -I/opt/vc/include -L/opt/vc/lib
LIBS = -lm -lbcm_host
TARGET = fm_transmitter

CPP=$(CCPREFIX)g++

all: main.o error_reporter.o wave_reader.o transmitter.o
	$(CPP) $(FLAGS) $(INCLUDES) $(LIBS) -o $(TARGET) main.o error_reporter.o wave_reader.o transmitter.o

wave_reader.o: wave_reader.cpp wave_reader.h
	$(CPP) $(FLAGS) $(INCLUDES) $(LIBS) -c wave_reader.cpp

error_reporter.o: error_reporter.cpp error_reporter.h
	$(CPP) $(FLAGS) $(INCLUDES) $(LIBS) -c error_reporter.cpp

transmitter.o: transmitter.cpp transmitter.h
	$(CPP) $(FLAGS) $(INCLUDES) $(LIBS) -c transmitter.cpp

main.o: main.cpp
	$(CPP) $(FLAGS) $(INCLUDES) $(LIBS) -c main.cpp

clean:
	rm *.o
