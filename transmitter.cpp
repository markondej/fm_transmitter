#include "transmitter.h"
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <exception>
#include <iostream>

#define ACCESS(base, offset) *(volatile int*)((int)base + offset)

Transmitter::Transmitter(double frequency)
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        std::cout << "Sudo privileges are required" << std::endl;
        throw std::exception();
    }

    void *peripheralsMap = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, 0x20000000);
    close(memFd);
    if (peripheralsMap == MAP_FAILED) {
        std::cout << "Cannot obtain access to peripherals (mmap error)" << std::endl;
        throw std::exception();
    }

    peripherals = (volatile unsigned*)peripheralsMap;

    ACCESS(peripherals, 0x00200000) = (ACCESS(peripherals, 0x00200000) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, 0x00101070) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;
}

void Transmitter::transmit(std::vector<unsigned int> *freqDivs, unsigned int sampleRate) {
    struct timeval time;

    gettimeofday(&time, NULL);
    unsigned int start_sec = (unsigned int)time.tv_sec;
    unsigned int start_usec = (unsigned int)time.tv_usec;
    unsigned int current_sec = 0, current_usec = 0;

    unsigned int offset = 0, dataOffset, dataLength = freqDivs->size();
    unsigned int *data = &(*freqDivs)[0];

    while (true) {
        dataOffset = (unsigned int)((unsigned long long)offset * (unsigned long long)sampleRate / 1000000);
        if (dataOffset >= dataLength) break;
        ACCESS(peripherals, 0x00101074) = (0x5A << 24) | data[dataOffset];

        usleep(15);

        gettimeofday(&time, NULL);
        current_sec = (unsigned int)time.tv_sec;
        current_usec = (unsigned int)time.tv_usec;
        offset = (current_sec - start_sec) * 1000000 + current_usec - start_usec;
    }
}

Transmitter::~Transmitter()
{
    ACCESS(peripherals, 0x00101070) = (0x5A << 24);
}
