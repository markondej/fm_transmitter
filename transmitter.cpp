/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2018, Marcin Kondej
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

#include "transmitter.h"
#include "error_reporter.h"
#include <bcm_host.h>
#include <sstream>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>

using std::ostringstream;
using std::vector;

#define GPIO_BASE 0x00200000
#define CLK0_BASE 0x00101070
#define CLK0DIV_BASE 0x00101074
#define TCNT_BASE 0x00003004

#define ACCESS(base, offset) *(volatile unsigned*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)

bool Transmitter::transmitting = false;
void* Transmitter::peripherals = NULL;

Transmitter::Transmitter() : forceStop(false)
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw ErrorReporter("Cannot open /dev/mem (permission denied)");
    }

    peripherals = mmap(NULL, bcm_host_get_peripheral_size(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, bcm_host_get_peripheral_address());
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw ErrorReporter("Cannot obtain access to peripherals (mmap error)");
    }
}

Transmitter::~Transmitter()
{
    munmap(peripherals, bcm_host_get_peripheral_size());
}

Transmitter* Transmitter::getInstance()
{
    static Transmitter instance;
    return &instance;
}

void Transmitter::play(WaveReader* reader, double frequency, unsigned char dmaChannel, bool loop)
{
    if (transmitting) {
        throw ErrorReporter("Cannot play, transmitter already in use");
    }

    transmitting = true;
    forceStop = false;

    PCMWaveHeader header = reader->getHeader();
    unsigned bufferFrames = (unsigned)((unsigned long long)header.sampleRate * BUFFER_TIME / 1000000);
    vector<float>* frames = reader->getFrames(bufferFrames, forceStop);
    if (frames == NULL) {
        return;
    }
    bool eof = frames->size() < bufferFrames;
    vector<float>* buffer = frames;

    unsigned frameOffset = 0;
    unsigned clockDivisor = (unsigned)((500 << 12) / frequency + 0.5);
    bool restart = false;

    void* transmitterParams[6] = {
        (void*)&restart,
        (void*)&buffer,
        (void*)&frameOffset,
        (void*)&clockDivisor,
        (void*)&header.sampleRate,
        (void*)&dmaChannel
    };

    pthread_t thread;
    int returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, (void*)transmitterParams);
    if (returnCode) {
        delete frames;
        ostringstream oss;
        oss << "Cannot create new thread (code: " << returnCode << ")";
        throw ErrorReporter(oss.str());
    }

    usleep(BUFFER_TIME / 2);

    bool isError = false;
    string errorMessage;

    try {
        while (!forceStop) {
            while (!eof && !forceStop) {
                if (buffer == NULL) {
                    if (!reader->setFrameOffset(frameOffset + bufferFrames)) {
                        break;
                    }
                    frames = reader->getFrames(bufferFrames, forceStop);
                    if (frames == NULL) {
                        forceStop = true;
                        break;
                    }
                    eof = frames->size() < bufferFrames;
                    buffer = frames;
                }
                usleep(BUFFER_TIME / 2);
            }
            if (loop && !forceStop) {
                frameOffset = 0;
                restart = true;
                if (!reader->setFrameOffset(0)) {
                    break;
                }
                frames = reader->getFrames(bufferFrames, forceStop);
                if (frames == NULL) {
                    break;
                }
                eof = frames->size() < bufferFrames;
                buffer = frames;
                usleep(BUFFER_TIME / 2);
            } else {
                forceStop = true;
            }
        }
    } catch (ErrorReporter &error) {
        errorMessage = error.what();
        isError = true;
    }
    transmitting = false;
    pthread_join(thread, NULL);
    if (isError) {
        throw ErrorReporter(errorMessage);
    }
}

void* Transmitter::transmit(void* params)
{
    bool* restart = (bool*)((void**)params)[0];
    vector<float>** buffer = (vector<float>**)((void**)params)[1];
    unsigned* frameOffset = (unsigned*)((void**)params)[2];
    unsigned clockDivisor = *(unsigned*)((void**)params)[3], sampleRate = *(unsigned*)((void**)params)[4];
    unsigned char dmaChannel = *(unsigned char*)((void**)params)[5];

    unsigned long long current, start, playbackStart;
    unsigned offset, length, prevOffset;
    vector<float>* frames = NULL;
    float* data;
    float value;

#ifndef NO_PREEMP
    float prevValue = 0.0;
    float preemp = 0.75 - 250000.0 / (float)(sampleRate * 75);
#endif

    ACCESS(peripherals, GPIO_BASE) = (ACCESS(peripherals, GPIO_BASE) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, CLK0_BASE) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;

    current = ACCESS64(peripherals, TCNT_BASE);
    playbackStart = current;

    while (transmitting) {
        start = current;

        while ((*buffer == NULL) && transmitting) {
            usleep(1);
            current = ACCESS64(peripherals, TCNT_BASE);
        }
        if (!transmitting) {
            break;
        }
        if (*restart) {
            playbackStart = current;
            start = current;
            *restart = false;
        }
        frames = *buffer;
        *frameOffset = (current - playbackStart) * (sampleRate) / 1000000;
        *buffer = NULL;

        offset = (current - start) * (sampleRate) / 1000000;

        length = frames->size();
        data = &(*frames)[0];

        while (true) {
            if (offset >= length) {
                break;
            }

            prevOffset = offset;
            value = data[offset];

#ifndef NO_PREEMP
            value = value + (value - prevValue) * preemp;
            value = (value < -1.0) ? -1.0 : ((value > 1.0) ? 1.0 : value);
#endif

            ACCESS(peripherals, CLK0DIV_BASE) = (0x5A << 24) | ((clockDivisor) - (int)(round(value * 16.0)));
            while (offset == prevOffset) {
                asm("nop");
                current = ACCESS64(peripherals, TCNT_BASE);
                offset = (current - start) * (sampleRate) / 1000000;
            }
#ifndef NO_PREEMP
            prevValue = value;
#endif
        }

        delete frames;
    }

    ACCESS(peripherals, CLK0_BASE) = (0x5A << 24);

    return NULL;
}

void Transmitter::stop()
{
    forceStop = true;
}
