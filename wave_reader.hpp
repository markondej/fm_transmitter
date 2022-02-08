/*
    FM Transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2021, Marcin Kondej
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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#define WAVE_FORMAT_PCM 0x0001

struct WaveHeader
{
    uint8_t chunkID[4];
    uint32_t chunkSize;
    uint8_t format[4];
    uint8_t subchunk1ID[4];
    uint32_t subchunk1Size;
    uint16_t audioFormat;
    uint16_t channels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    uint8_t subchunk2ID[4];
    uint32_t subchunk2Size;
};

class Sample
{
public:
    Sample(uint8_t *data, unsigned channels, unsigned bitsPerChannel);
    float GetMonoValue() const;
protected:
    float value;
};

class WaveReader
{
    public:
        WaveReader(const std::string &filename, bool &stop);
        virtual ~WaveReader();
        WaveReader(const WaveReader &) = delete;
        WaveReader(WaveReader &&) = delete;
        WaveReader &operator=(const WaveReader &) = delete;
        std::string GetFilename() const;
        const WaveHeader &GetHeader() const;
        std::vector<Sample> GetSamples(unsigned quantity, bool &stop);
        bool SetSampleOffset(unsigned offset);
    private:
        std::vector<uint8_t> ReadData(unsigned bytesToRead, bool headerBytes, bool &stop);

        std::string filename;
        WaveHeader header;
        unsigned dataOffset, headerOffset, currentDataOffset;
        int fileDescriptor;
};
