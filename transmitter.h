#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <vector>

class Transmitter
{
    public:
        Transmitter(double frequency);
        virtual ~Transmitter();
        void transmit(std::vector<float> *samples, unsigned int sampleRate);
    private:
        unsigned int clockDivisor;
        volatile unsigned *peripherals;
};

#endif // TRANSMITTER_H
