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

#define GPIO_BASE 0x00200000
#define CLK0_BASE 0x00101070
#define CLK0DIV_BASE 0x00101074
#define TCNT_BASE 0x00003004

#define ACCESS(base, offset) *(volatile unsigned int*)((int)base + offset)
#define ACCESS64(base, offset) *(volatile unsigned long long*)((int)base + offset)

bool Transmitter::isTransmitting = false;

Transmitter::Transmitter(string filename, double frequency) :
    readStdin(filename == "-")
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

    AudioFormat *audioFormat;
    if (!readStdin) {
        waveReader = new WaveReader(filename);
        audioFormat = waveReader->getFormat();
    } else {
        stdinReader = StdinReader::getInstance();
        audioFormat = stdinReader->getFormat();
        usleep(700000);
    }
    format = *audioFormat;

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

    frameOffset = 0;
    isTransmitting = true;
    pthread_t thread;

    unsigned int bufferFrames = (unsigned int)((unsigned long long)format.sampleRate * BUFFER_TIME / 1000000);

    buffer = (!readStdin) ? waveReader->getFrames(bufferFrames, frameOffset) : stdinReader->getFrames(bufferFrames);

    vector<void*> params;
    params.push_back((void*)&format.sampleRate);
    params.push_back((void*)&clockDivisor);
    params.push_back((void*)&frameOffset);
    params.push_back((void*)&buffer);
    params.push_back((void*)peripherals);

    int returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, (void*)&params);
    if (returnCode) {
        oss << "Cannot create new thread (code: " << returnCode << ")";
        errorMessage = oss.str();
        throw exception();
    }

    usleep(BUFFER_TIME / 2);

    while(readStdin || !waveReader->isEnd()) {
        if (buffer == NULL) {
            buffer = (!readStdin) ? waveReader->getFrames(bufferFrames, frameOffset + bufferFrames) : stdinReader->getFrames(bufferFrames);
        }
        usleep(BUFFER_TIME / 2);
    }

    isTransmitting = false;
    pthread_join(thread, NULL);
}

void Transmitter::transmit(void *params)
{
    unsigned long long current, start, playbackStart;
    unsigned int offset, length, temp;
	float prevValue = 0.0, value = 0.0;
    vector<float> *frames = NULL;
    float *data;

    unsigned int *sampleRate = (unsigned int*)(*((vector<void*>*)params))[0];
    unsigned int *clockDivisor = (unsigned int*)(*((vector<void*>*)params))[1];
    unsigned int *frameOffset = (unsigned int*)(*((vector<void*>*)params))[2];
    vector<float> **buffer = (vector<float>**)(*((vector<void*>*)params))[3];
    volatile unsigned *peripherals = (volatile unsigned*)(*((vector<void*>*)params))[4];

	float preemp = 0.75 - 250000.0 / (float)(*sampleRate * 75);

    ACCESS(peripherals, GPIO_BASE) = (ACCESS(peripherals, GPIO_BASE) & 0xFFFF8FFF) | (0x01 << 14);
    ACCESS(peripherals, CLK0_BASE) = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;

    playbackStart = ACCESS64(peripherals, TCNT_BASE);
    current = playbackStart;
    start = playbackStart;

    while (isTransmitting) {
        while (*buffer == NULL) {
            usleep(1);
            current = ACCESS64(peripherals, TCNT_BASE);
        }
        frames = *buffer;
        *frameOffset = (current - playbackStart) * (*sampleRate) / 1000000;
        *buffer = NULL;

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

			/* pre emphasis */
			value = value + (value - prevValue) * preemp;

			value = (value < -1.0) ? -1.0 : ((value > 1.0) ? 1.0 : value);
            ACCESS(peripherals, CLK0DIV_BASE) = (0x5A << 24) | ((*clockDivisor) - (int)(round(value * 16.0)));
            while (temp >= offset) {
                usleep(1);
                current = ACCESS64(peripherals, TCNT_BASE);
                offset = (current - start) * (*sampleRate) / 1000000;
            }
			prevValue = value;
        }

		start = ACCESS64(peripherals, TCNT_BASE);
        delete frames;
    }

    ACCESS(peripherals, CLK0_BASE) = (0x5A << 24);
}

Transmitter::~Transmitter()
{
    munmap(peripherals, 0x002FFFFF);
    if (!readStdin) {
        delete waveReader;
    } else {
        delete stdinReader;
    }
}

AudioFormat& Transmitter::getFormat()
{
    return format;
}
