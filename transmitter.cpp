/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2015, Marcin Kondej
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
#include "wave_reader.h"
#include <sstream>
#include <cmath>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

using std::ostringstream;

#define GPIO_BASE 0x00200000
#define CLK0_BASE 0x00101070
#define CLK0DIV_BASE 0x00101074
#define TCNT_BASE 0x00003004

#define ACCESS(base, offset) *(volatile unsigned*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)

bool Transmitter::transmitting = false;
bool Transmitter::restart = false;
unsigned Transmitter::clockDivisor = 0;
unsigned Transmitter::frameOffset = 0;
vector<float>* Transmitter::buffer = NULL;
void* Transmitter::peripherals = NULL;

Transmitter::Transmitter()
{
    bool isBcm2835 = true;

    FILE* pipe = popen("uname -m", "r");
    if (pipe) {
        char buffer[64];
        string machine = "";
        while (!feof(pipe)) {
            if (fgets(buffer, 64, pipe)) {
                machine += buffer;
            }
        }
        pclose(pipe);

        machine = machine.substr(0, machine.length() - 1);
        if (machine != "armv6l") {
            isBcm2835 = false;
        }
    }

    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw ErrorReporter("Cannot open /dev/mem (permission denied)");
    }

    peripherals = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, isBcm2835 ? 0x20000000 : 0x3F000000);
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw ErrorReporter("Cannot obtain access to peripherals (mmap error)");
    }
}

Transmitter::~Transmitter()
{
    munmap(peripherals, 0x002FFFFF);
}

Transmitter* Transmitter::getInstance()
{
    static Transmitter instance;
    return &instance;
}

void Transmitter::play(string filename, double frequency, bool loop)
{
    if (transmitting) {
        throw ErrorReporter("Cannot play, transmitter already in use");
    }

    transmitting = true;
    forceStop = false;

    WaveReader* reader = new WaveReader(filename != "-" ? filename : string(), forceStop);
    AudioFormat* format = reader->getFormat();

    clockDivisor = (unsigned)((500 << 12) / frequency + 0.5);
    unsigned bufferFrames = (unsigned)((unsigned long long)format->sampleRate * BUFFER_TIME / 1000000);

    frameOffset = 0;
    restart = false;

    try {
        vector<float>* frames = reader->getFrames(bufferFrames, forceStop);
        if (frames == NULL) {
            delete format;
            delete reader;
            return;
        }
        eof = frames->size() < bufferFrames;
        buffer = frames;

        pthread_t thread;
        void* params = (void*)&format->sampleRate;

        int returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, params);
        if (returnCode) {
            delete reader;
            delete format;
            delete frames;
            ostringstream oss;
            oss << "Cannot create new thread (code: " << returnCode << ")";
            throw ErrorReporter(oss.str());
        }

        usleep(BUFFER_TIME / 2);

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
        transmitting = false;

        pthread_join(thread, NULL);
   } catch (ErrorReporter &error) {
        delete reader;
        delete format;
        throw error;
    }

    delete reader;
    delete format;
 }

void* Transmitter::transmit(void* params)
{
    unsigned long long current, start, playbackStart;
    unsigned offset, length, temp;
    vector<float>* frames = NULL;
    float* data;
    float value;
#ifndef NO_PREEMP
    float prevValue = 0.0;
#endif

    unsigned sampleRate = *(unsigned*)(params);

#ifndef NO_PREEMP
    float preemp = 0.75 - 250000.0 / (float)(sampleRate * 75);
#endif

    ACCESS(peripherals, GPIO_BASE) = (ACCESS(peripherals, GPIO_BASE) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, CLK0_BASE) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;

    playbackStart = ACCESS64(peripherals, TCNT_BASE);
    current = playbackStart;

    while (transmitting) {
        start = current;
        while ((buffer == NULL) && transmitting) {
            usleep(1);
            current = ACCESS64(peripherals, TCNT_BASE);
        }
        if (!transmitting) {
            break;
        }
        if (restart) {
            playbackStart = current;
            start = current;
            restart = false;
        }
        frames = buffer;
        frameOffset = (current - playbackStart) * (sampleRate) / 1000000;
        buffer = NULL;

        length = frames->size();
        data = &(*frames)[0];

        offset = 0;

        while (true) {
            temp = offset;
            if (offset >= length) {
                offset -= length;
                break;
            }

            value = data[offset];

#ifndef NO_PREEMP
            value = value + (value - prevValue) * preemp;
            value = (value < -1.0) ? -1.0 : ((value > 1.0) ? 1.0 : value);
#endif

            ACCESS(peripherals, CLK0DIV_BASE) = (0x5A << 24) | ((clockDivisor) - (int)(round(value * 16.0)));
            while (temp >= offset) {
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
