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
#include "error_reporter.h"
#include <sstream>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

using std::ostringstream;

bool StdinReader::doStop = false;
bool StdinReader::isReading = false;
bool StdinReader::isDataAccess = false;
vector<char> StdinReader::stream;

StdinReader::StdinReader()
{
    int returnCode = pthread_create(&thread, NULL, &StdinReader::readStdin, NULL);
    if (returnCode) {
        ostringstream oss;
        oss << "Cannot create new thread (code: " << returnCode << ")";
        throw ErrorReporter(oss.str());
    }

    while (!isReading) {
        usleep(1);
    }
}

StdinReader::~StdinReader()
{
    doStop = true;
    pthread_join(thread, NULL);
}

StdinReader* StdinReader::getInstance()
{
    static StdinReader instance;
    return &instance;
}

void *StdinReader::readStdin(void *params)
{
    long flag = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flag | O_NONBLOCK);

    char *readBuffer = new char[1024];
    while (!doStop) {
        isReading = true;

        while (isDataAccess && !doStop) {
            usleep(1);
        }
        if (doStop) {
            break;
        }
        unsigned streamSize = stream.size();
        if (streamSize < MAX_STREAM_SIZE) {
            int bytes = read(STDIN_FILENO, readBuffer, (streamSize + 1024 > MAX_STREAM_SIZE) ? MAX_STREAM_SIZE - streamSize : 1024);
            if (bytes > 0) {
                stream.insert(stream.end(), readBuffer, readBuffer + bytes);
            }
        }

        isReading = false;
        usleep(1);
    }
    delete readBuffer;

    return NULL;
}

vector<float>* StdinReader::getFrames(unsigned frameCount, bool &forceStop)
{
    while (isReading && !forceStop) {
        usleep(1);
    }
    if (forceStop) {
        doStop = true;
        return NULL;
    }

    isDataAccess = true;

    unsigned offset, bytesToRead, bytesPerFrame;
    unsigned streamSize = stream.size();
    if (!streamSize) {
        isDataAccess = false;
        return NULL;
    }

    vector<float> *frames = new vector<float>();
    bytesPerFrame = (STREAM_BITS_PER_SAMPLE >> 3) * STREAM_CHANNELS;
    bytesToRead = frameCount * bytesPerFrame;

    if (bytesToRead > streamSize) {
        bytesToRead = streamSize - streamSize % bytesPerFrame;
        frameCount = bytesToRead / bytesPerFrame;
    }

    vector<char> data;
    data.resize(bytesToRead);
    memcpy(&(data[0]), &(stream[0]), bytesToRead);
    stream.erase(stream.begin(), stream.begin() + bytesToRead);
    isDataAccess = false;

    for (unsigned i = 0; i < frameCount; i++) {
        offset = bytesPerFrame * i;
        if (STREAM_CHANNELS != 1) {
            if (STREAM_BITS_PER_SAMPLE != 8) {
                frames->push_back(((int)(signed char)data[offset + 1] + (int)(signed char)data[offset + 3]) / (float)0x100);
            } else {
                frames->push_back(((int)data[offset] + (int)data[offset + 1]) / (float)0x100 - 1.0f);
            }
        } else {
            if (STREAM_BITS_PER_SAMPLE != 8) {
                frames->push_back((signed char)data[offset + 1] / (float)0x80);
            } else {
                frames->push_back(data[offset] / (float)0x80 - 1.0f);
            }
        }
    }

    return frames;
}

AudioFormat* StdinReader::getFormat()
{
    AudioFormat* format = new AudioFormat;
    format->sampleRate = STREAM_SAMPLE_RATE;
    format->bitsPerSample = STREAM_BITS_PER_SAMPLE;
    format->channels = STREAM_CHANNELS;
    return format;
}
