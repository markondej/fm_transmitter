CROSS_COMPILE =
CPP = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
CFLAGS = -Wall -O3
CXXFLAGS = $(CFLAGS) -std=c++11
OBJS = mailbox.o wave_reader.o transmitter.o fm_transmitter.o
LIBS = -lm -lpthread
BINARY = fm_transmitter

VERSION = 0.9.7
DEFINES = -DVERSION=\"$(VERSION)\"

ifdef GPIO21
	TRANSMITTER += -DGPIO21
endif

.PHONY: clean

$(BINARY): $(OBJS)
	$(CXX) -o $(BINARY) $(OBJS) $(LIBS)

mailbox.o: mailbox.c
	$(CPP) $(CFLAGS) -c $<

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(DEFINES) -c $<

clean:
	rm *.o
