/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2019, Marcin Kondej
    All rights reserved.

    See https://github.com/markondej/fm_transmitter

    Redistribution and use in source and binary forms, with or without modification, are
    permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors may be
    used to endorse or promote products derived from this software without specific
    prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
    SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
    TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
    BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TRANSMITTER_HPP
#define TRANSMITTER_HPP

#include "wave_reader.hpp"
#include <mutex>

struct AllocatedMemory;
struct PWMRegisters;
struct DMARegisters;
struct DMAControllBlock;
struct ClockRegisters;

class Transmitter
{
    public:
        virtual ~Transmitter();
        Transmitter(const Transmitter &source) = delete;
        Transmitter &operator=(const Transmitter &source) = delete;
        static Transmitter &getInstance();
        void transmit(WaveReader &reader, float frequency, float bandwidth, uint8_t dmaChannel, bool preserveCarrierOnExit);
        void stop();
    private:
        Transmitter();
        uint32_t getPeripheralsVirtBaseAddress();
        uint32_t getPeripheralsSize();
        float getSourceFreq();
        uint32_t getPeripheralPhysAddress(volatile void *object);
        static uint32_t getPeripheralVirtAddress(uint32_t offset);
        uint32_t getMemoryPhysAddress(AllocatedMemory &memory, volatile void *object);
        AllocatedMemory allocateMemory(uint32_t size);
        void freeMemory(AllocatedMemory &memory);
        volatile PWMRegisters *initPwmController();
        void closePwmController(volatile PWMRegisters *pwm);
        volatile DMARegisters *startDma(AllocatedMemory &memory, volatile DMAControllBlock *dmaCb, uint8_t dmaChannel);
        void closeDma(volatile DMARegisters *dma);
        volatile ClockRegisters *initClockOutput();
        void closeClockOutput(volatile ClockRegisters *clock);
        void transmitViaCpu(WaveReader &reader, uint32_t bufferSize);
        void transmitViaDma(WaveReader &reader, uint32_t bufferSize, uint8_t dmaChannel);
        static void transmitThread();

        bool preserveCarrier;

        static void *peripherals;
        static bool transmitting;
        static uint32_t sampleOffset, clockDivisor, divisorRange, sampleRate;
        static volatile ClockRegisters *output;
        static std::vector<Sample> samples;
        static std::mutex samplesMutex;
};

#endif // TRANSMITTER_HPP
