#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <vector>

class Transmitter
{
    public:
        Transmitter(double frequency);
        virtual ~Transmitter();
        void transmit(std::vector<unsigned int> *freqDivs, unsigned int sampleRate);
    private:
        unsigned int minFreqDiv, maxFreqDiv;
        volatile unsigned *peripherals;
};

#endif // TRANSMITTER_H
