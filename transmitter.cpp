#include "transmitter.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <exception>
#include <iostream>

#define ACCESS(base, offset) *(volatile int*)((int)base + offset)

Transmitter::Transmitter(double frequency)
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        std::cout << "Error: sudo privileges are required" << std::endl;
        throw std::exception();
    }

    void *peripheralsMap = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, 0x20000000);
    close(memFd);
    if (peripheralsMap == MAP_FAILED) {
        std::cout << "Error: cannot obtain access to peripherals (mmap error)" << std::endl;
        throw std::exception();
    }

    peripherals = (volatile unsigned*)peripheralsMap;

    ACCESS(peripherals, 0x00200000) = (ACCESS(peripherals, 0x00200000) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, 0x00101070) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;
}

void Transmitter::transmit(std::vector<unsigned int> *freqDivs, unsigned int sampleRate) {
    unsigned int offset = 0, length = freqDivs->size(), temp;
    unsigned int *data = &(*freqDivs)[0];

    unsigned long long current = 0;
    unsigned long long start = ((unsigned long long)ACCESS(peripherals, 0x00003008) << 32) | ACCESS(peripherals, 0x00003004);

    while (true) {
        temp = offset;
        if (offset >= length) break;

        ACCESS(peripherals, 0x00101074) = (0x5A << 24) | data[offset];

        while (temp == offset) {
            usleep(1);

            current = ((unsigned long long)ACCESS(peripherals, 0x00003008) << 32) | ACCESS(peripherals, 0x00003004);
            offset = (unsigned int)((current - start) * (unsigned long long)sampleRate / 1000000);
        }
    }
}

Transmitter::~Transmitter()
{
    ACCESS(peripherals, 0x00101070) = (0x5A << 24);
}
