#pragma once

#include "wave_reader.hpp"
#include <mutex>

class ClockOutput;
class Resampler;

class Transmitter
{
    public:
        Transmitter();
        virtual ~Transmitter();
        Transmitter(const Transmitter &) = delete;
        Transmitter(Transmitter &&) = delete;
        Transmitter &operator=(const Transmitter &) = delete;
        void Transmit(WaveReader &reader, float frequency, float bandwidth, unsigned dmaChannel, bool resample, bool preserveCarrier);
        void Stop();
    private:
        void TransmitViaCpu(Resampler &resampler, ClockOutput &output, unsigned clockDivisor);
        void TransmitViaDma(Resampler &resampler, ClockOutput &output, unsigned clockDivisor, unsigned dmaChannel);
        static void TransmitterThread(Transmitter *instance, ClockOutput *output, unsigned sampleRate, unsigned clockDivisor, std::size_t *frame, std::vector<int> *samples, bool *stop);

        ClockOutput *output;
        std::mutex access;
        bool stop;
};
