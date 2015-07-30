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

#include "stdin_reader.h"
#include <exception>
#include <sstream>
#include <unistd.h>
#include <string.h>

using std::exception;
using std::ostringstream;

StdinReader::StdinReader()
{
    ostringstream oss;

    doStop = true;
    isDataAccess = false;
    isReading = false;

    vector<void*> params;
    params.push_back((void*)&buffer);
    params.push_back((void*)&doStop);
    params.push_back((void*)&isDataAccess);
    params.push_back((void*)&isReading);

    int returnCode = pthread_create(&thread, NULL, &StdinReader::readStdin, (void*)&params);
    if (returnCode) {
        oss << "Cannot create new thread (code: " << returnCode << ")";
        errorMessage = oss.str();
        throw exception();
    }

    while (doStop) {
        usleep(1);
    }
}

StdinReader::~StdinReader()
{
    pthread_join(thread, NULL);
}

StdinReader *StdinReader::getInstance()
{
    static StdinReader instance;
    return &instance;
}

void StdinReader::readStdin(void *params)
{
    vector<char> *buffer = (vector<char>*)(*((vector<void*>*)params))[0];
    bool *doStop = (bool*)(*((vector<void*>*)params))[1];
    bool *isDataAccess = (bool*)(*((vector<void*>*)params))[2];
    bool *isReading = (bool*)(*((vector<void*>*)params))[3];

    *doStop = false;
    char *readBuffer = new char[BUFFER_SIZE];
    while(!*doStop) {
        *isReading = true;

        while (*isDataAccess) {
            usleep(1);
        }

        int bytes = read(STDIN_FILENO, readBuffer, BUFFER_SIZE);
        buffer->insert(buffer->end(), readBuffer, readBuffer + bytes);

        *isReading = false;
        usleep(1);

    }

    delete readBuffer;
}

vector<float> *StdinReader::getFrames(unsigned int frameCount)
{
    unsigned int bytesToRead, bufferSize, bytesPerFrame, offset;
    vector<float> *frames = new vector<float>();

    while (isReading) {
        usleep(1);
    }
    isDataAccess = true;

    bufferSize = buffer.size();
    bytesPerFrame = (BITS_PER_SAMPLE >> 3) * CHANNELS;
    bytesToRead = frameCount * bytesPerFrame;

    if (bytesToRead > bufferSize) {
        bytesToRead = bufferSize - bufferSize % bytesPerFrame;
        frameCount = bytesToRead / bytesPerFrame;
    }

    vector<char> data;
    data.resize(bytesToRead);
    memcpy(&(data[0]), &(buffer[0]), bytesToRead);
    buffer.erase(buffer.begin(), buffer.begin() + bytesToRead);
    isDataAccess = false;

    for (unsigned int i = 0; i < frameCount; i++) {
        offset = bytesPerFrame * i;
        if (CHANNELS != 1) {
            if (BITS_PER_SAMPLE != 8) {
                frames->push_back(((int)(signed char)data[offset + 1] + (int)(signed char)data[offset + 3]) / (float)0x100);
            } else {
                frames->push_back(((int)data[offset] + (int)data[offset + 1]) / (float)0x100 - 1.0f);
            }
        } else {
            if (BITS_PER_SAMPLE != 8) {
                frames->push_back((signed char)data[offset + 1] / (float)0x80);
            } else {
                frames->push_back(data[offset] / (float)0x80 - 1.0f);
            }
        }
    }

    return frames;
}

AudioFormat *StdinReader::getFormat()
{
    AudioFormat *format = new AudioFormat;
    format->sampleRate = SAMPLE_RATE;
    format->bitsPerSample = BITS_PER_SAMPLE;
    format->channels = CHANNELS;
    return format;
}
