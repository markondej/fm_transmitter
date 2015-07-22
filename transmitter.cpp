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
#include <exception>
#include <sstream>
#include <cmath>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>

using std::exception;
using std::ostringstream;

#define ACCESS(base, offset) *(volatile unsigned int*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)

bool Transmitter::isTransmitting = false;
vector<float> *Transmitter::buffer = NULL;
unsigned int Transmitter::frameOffset = 0;

Transmitter::Transmitter(string filename, double frequency)
{
    ostringstream oss;
    bool isBcm2835 = true;

    FILE *pipe = popen("uname -m", "r");
    if (pipe) {
        char buffer[64];
        string machine = "";
        while (!feof(pipe)) {
            if (fgets(buffer, 64, pipe)) {
                machine += buffer;
            }
        }
        pclose(pipe);

        if (machine != "armv6l\n") {
            isBcm2835 = false;
        }
    }

    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        oss << "Cannot open /dev/mem (permission denied)";
        errorMessage = oss.str();
        throw exception();
    }

    void *peripheralsMap = mmap(NULL, 0x002FFFFF, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, isBcm2835 ? 0x20000000 : 0x3F000000);
    close(memFd);
    if (peripheralsMap == MAP_FAILED) {
        oss << "Cannot obtain access to peripherals (mmap error)";
        errorMessage = oss.str();
        throw exception();
    }

    peripherals = (volatile unsigned*)peripheralsMap;

    waveReader = new WaveReader(filename);

    format.sampleRate = waveReader->getHeader()->sampleRate;
    format.channels = waveReader->getHeader()->channels;
    format.bitsPerSample = waveReader->getHeader()->bitsPerSample;

    ACCESS(peripherals, 0x00200000) = (ACCESS(peripherals, 0x00200000) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, 0x00101070) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;

    clockDivisor = (unsigned int)((500 << 12) / frequency + 0.5);
}

void Transmitter::play()
{
    ostringstream oss;

    if (isTransmitting) {
        oss << "Cannot play, transmitter already in use";
        errorMessage = oss.str();
        throw exception();
    }

    isTransmitting = true;
    pthread_t thread;
    int returnCode;

    frameOffset = 0;

    unsigned int bufferFrames = format.sampleRate * BUFFER_TIME / 100000;

    buffer = waveReader->getFrames(bufferFrames, frameOffset);

    void *params[3];
    params[0] = &format.sampleRate;
    params[1] = &clockDivisor;
    params[2] = peripherals;

    returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, (void*)params);
    if (returnCode) {
        oss << "Cannot create new thread (code: " << returnCode << ")";
        errorMessage = oss.str();
        throw exception();
    }

    while(!waveReader->isEnd()) {
        if (buffer == NULL) {
            buffer = waveReader->getFrames(bufferFrames, frameOffset + bufferFrames);
        }
        usleep(BUFFER_TIME / 2);
    }

    isTransmitting = false;
    pthread_join(thread, NULL);
}

void Transmitter::transmit(void *params)
{
    unsigned long long current, start, playbackStart;
    unsigned int offset = 0, length, temp;
    vector<float> *frames;
    float *data;

    unsigned int sampleRate = *((unsigned int**)params)[0];
    unsigned int clockDivisor = *((unsigned int**)params)[1];
    volatile unsigned *peripherals = ((unsigned int**)params)[2];

    playbackStart = ACCESS64(peripherals, 0x00003004);
    current = playbackStart;
    start = playbackStart;

    while (isTransmitting) {
        while (buffer == NULL) {
            usleep(1);
            current = ACCESS64(peripherals, 0x00003004);
        }
        frames = buffer;
        frameOffset = (current - playbackStart) * sampleRate / 1000000;
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

            ACCESS(peripherals, 0x00101074) = (0x5A << 24) | clockDivisor - (int)(round(data[offset] * 16.0));

            while (temp >= offset) {
                usleep(1);

                current = ACCESS64(peripherals, 0x00003004);
                offset = (current - start) * sampleRate / 1000000;
            }
        }

        start = ACCESS64(peripherals, 0x00003004);

        delete frames;
    }
}

Transmitter::~Transmitter()
{
    ACCESS(peripherals, 0x00101070) = (0x5A << 24);
    munmap(peripherals, 0x002FFFFF);
    peripherals = NULL;
    delete waveReader;
}

AudioFormat *Transmitter::getFormat()
{
    return &format;
}
