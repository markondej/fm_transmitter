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

#include "wave_reader.h"
#include "error_reporter.h"
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

using std::ostringstream;
using std::exception;

WaveReader::WaveReader(string filename, bool &forceStop) :
    filename(filename), currentOffset(0)
{
    char* headerData = (char*)((void*)&header);
    vector<char>* data;
    unsigned bytesToRead, headerOffset;
    ostringstream oss;

    if (!filename.empty()) {
        fileDescriptor = open(filename.c_str(), O_RDONLY);
    } else {
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        fileDescriptor = STDIN_FILENO;
    }

    if (fileDescriptor == -1) {
        oss << "Cannot open " << getFilename() << ", file does not exist";
        throw ErrorReporter(oss.str());
    }

    try {
        bytesToRead = sizeof(PCMWaveHeader::chunkID) + sizeof(PCMWaveHeader::chunkSize) +
            sizeof(PCMWaveHeader::format);
        data = readData(bytesToRead, true, forceStop);
        if (data == NULL) {
            throw ErrorReporter("Cannot obtain header, program interrupted");
        }
        memcpy(headerData, &(*data)[0], bytesToRead);
        headerOffset = bytesToRead;
        delete data;

        if ((string(header.chunkID, 4) != string("RIFF")) || (string(header.format, 4) != string("WAVE"))) {
            oss << "Error while opening " << getFilename() << ", WAVE file expected";
            throw ErrorReporter(oss.str());
        }

        bytesToRead = sizeof(PCMWaveHeader::subchunk1ID) + sizeof(PCMWaveHeader::subchunk1Size);
        data = readData(bytesToRead, true, forceStop);
        if (data == NULL) {
            throw ErrorReporter("Cannot obtain header, program interrupted");
        }
        memcpy(&headerData[headerOffset], &(*data)[0], bytesToRead);
        headerOffset += bytesToRead;
        delete data;

        unsigned subchunk1MinSize = sizeof(PCMWaveHeader::audioFormat) + sizeof(PCMWaveHeader::channels) +
            sizeof(PCMWaveHeader::sampleRate) + sizeof(PCMWaveHeader::byteRate) + sizeof(PCMWaveHeader::blockAlign) +
            sizeof(PCMWaveHeader::bitsPerSample);
        if ((string(header.subchunk1ID, 4) != string("fmt ")) || (header.subchunk1Size < subchunk1MinSize)) {
            oss << "Error while opening " << getFilename() << ", data corrupted";
            throw ErrorReporter(oss.str());
        }

        data = readData(header.subchunk1Size, true, forceStop);
        if (data == NULL) {
            throw ErrorReporter("Cannot obtain header, program interrupted");
        }
        memcpy(&headerData[headerOffset], &(*data)[0], subchunk1MinSize);
        headerOffset += subchunk1MinSize;
        delete data;

        if ((header.audioFormat != WAVE_FORMAT_PCM) ||
            (header.byteRate != (header.bitsPerSample >> 3) * header.channels * header.sampleRate) ||
            (header.blockAlign != (header.bitsPerSample >> 3) * header.channels) ||
            (((header.bitsPerSample >> 3) != 1) && ((header.bitsPerSample >> 3) != 2))) {
            oss << "Error while opening " << getFilename() << ", unsupported WAVE format";
            throw ErrorReporter(oss.str());
        }

        bytesToRead = sizeof(PCMWaveHeader::subchunk2ID) + sizeof(PCMWaveHeader::subchunk2Size);
        data = readData(bytesToRead, true, forceStop);
        if (data == NULL) {
            throw ErrorReporter("Cannot obtain header, program interrupted");
        }
        memcpy(&headerData[headerOffset], &(*data)[0], bytesToRead);
        headerOffset += bytesToRead;
        delete data;

        if (string(header.subchunk2ID, 4) != string("data")) {
            oss << "Error while opening " << getFilename() << ", data corrupted";
            throw ErrorReporter(oss.str());
        }
    } catch (ErrorReporter &error) {
        if (fileDescriptor != STDIN_FILENO) {
            close(fileDescriptor);
        }
        throw error;
    }

    if (fileDescriptor != STDIN_FILENO) {
        dataOffset = lseek(fileDescriptor, 0, SEEK_CUR);
    }
}

WaveReader::~WaveReader()
{
    if (fileDescriptor != STDIN_FILENO) {
        close(fileDescriptor);
    }
}

vector<char>* WaveReader::readData(unsigned bytesToRead, bool headerBytes, bool &forceStop)
{
    unsigned bytesRead = 0;
    vector<char>* data = new vector<char>();
    data->resize(bytesToRead);

    while ((bytesRead < bytesToRead) && !forceStop) {
        int bytes = read(fileDescriptor, &(*data)[bytesRead], bytesToRead - bytesRead);
        if (((bytes == -1) && ((fileDescriptor != STDIN_FILENO) || (errno != EAGAIN))) ||
            (((unsigned)bytes < bytesToRead) && headerBytes && (fileDescriptor != STDIN_FILENO))) {
            delete data;

            if (fileDescriptor != STDIN_FILENO) {
                close(fileDescriptor);
            }

            ostringstream oss;
            oss << "Error while reading " << getFilename() << ", file is corrupted";
            throw ErrorReporter(oss.str());
        }
        if (bytes > 0) {
            bytesRead += bytes;
        }
        if (bytesRead < bytesToRead) {
            if (fileDescriptor != STDIN_FILENO) {
                data->resize(bytes);
                break;
            }
            usleep(1);
        }
    }

    if (!headerBytes) {
        currentOffset += bytesRead;
    }

    if (forceStop) {
        delete data;
        data = NULL;
    }

    return data;
}

vector<float>* WaveReader::getFrames(unsigned frameCount, bool &forceStop) {
    unsigned bytesToRead, bytesLeft, bytesPerFrame, offset;
    vector<float>* frames = new vector<float>();
    vector<char>* data;

    bytesPerFrame = (header.bitsPerSample >> 3) * header.channels;
    bytesToRead = frameCount * bytesPerFrame;
    bytesLeft = header.subchunk2Size - currentOffset;
    if (bytesToRead > bytesLeft) {
        bytesToRead = bytesLeft - bytesLeft % bytesPerFrame;
        frameCount = bytesToRead / bytesPerFrame;
    }

    try {
        data = readData(bytesToRead, false, forceStop);
    } catch (ErrorReporter &error) {
        delete frames;
        throw error;
    }
    if (data == NULL) {
        delete frames;
        return NULL;
    }
    if (data->size() < bytesToRead) {
        frameCount = data->size() / bytesPerFrame;
    }

    for (unsigned i = 0; i < frameCount; i++) {
        offset = bytesPerFrame * i;
        if (header.channels != 1) {
            if (header.bitsPerSample != 8) {
                frames->push_back(((int)(signed char)(*data)[offset + 1] + (int)(signed char)(*data)[offset + 3]) / (float)0x100);
            } else {
                frames->push_back(((int)(*data)[offset] + (int)(*data)[offset + 1]) / (float)0x100 - 1.0f);
            }
        } else {
            if (header.bitsPerSample != 8) {
                frames->push_back((signed char)(*data)[offset + 1] / (float)0x80);
            } else {
                frames->push_back((*data)[offset] / (float)0x80 - 1.0f);
            }
        }
    }

    delete data;
    return frames;
}

bool WaveReader::setFrameOffset(unsigned frameOffset) {
    if (fileDescriptor != STDIN_FILENO) {
        currentOffset = frameOffset * (header.bitsPerSample >> 3) * header.channels;
        if (lseek(fileDescriptor, dataOffset + currentOffset, SEEK_SET) == -1) {
            return false;
        }
    }
    return true;
}

string WaveReader::getFilename()
{
    return fileDescriptor != STDIN_FILENO ? filename : "STDIN";
}

AudioFormat* WaveReader::getFormat()
{
    AudioFormat* format = new AudioFormat;
    format->channels = header.channels;
    format->sampleRate = header.sampleRate;
    format->bitsPerSample = header.bitsPerSample;
    return format;
}
