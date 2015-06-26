#include "transmitter.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <exception>
#include <iostream>

#define ACCESS(base, offset) *(volatile unsigned int*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)

Transmitter::Transmitter(double frequency)
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        std::cout << "Error: sudo privileges are required" << std::endl;
        throw std::exception();
    }

    void *peripheralsMap = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, 0x3F000000);
    close(memFd);
    if (peripheralsMap == MAP_FAILED) {
        std::cout << "Error: cannot obtain access to peripherals (mmap error)" << std::endl;
        throw std::exception();
    }

    peripherals = (volatile unsigned*)peripheralsMap;

    ACCESS(peripherals, 0x00200000) = (ACCESS(peripherals, 0x00200000) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, 0x00101070) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;

    clockDivisor = (unsigned int)((500 << 12) / frequency + 0.5);
}

void Transmitter::transmit(std::vector<float> *samples, unsigned int sampleRate) {
    unsigned int offset = 0, length = samples->size(), temp;
    float *data = &(*samples)[0];

    unsigned long long current = 0;
    unsigned long long start = ACCESS64(peripherals, 0x00003004);

    while (true) {
        temp = offset;
        if (offset >= length) break;

        ACCESS(peripherals, 0x00101074) = (0x5A << 24) | clockDivisor - (int)(round(data[offset] * 10.0));

        while (temp >= offset) {
            usleep(1);

            current = ACCESS64(peripherals, 0x00003004);
            offset = (unsigned int)((current - start) * sampleRate / 1000000);
        }
    }
}

Transmitter::~Transmitter()
{
    ACCESS(peripherals, 0x00101070) = (0x5A << 24);
}
