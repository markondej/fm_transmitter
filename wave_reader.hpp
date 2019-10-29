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

#ifndef WAVE_READER_HPP
#define WAVE_READER_HPP

#include "pcm_wave_header.hpp"
#include "sample.hpp"
#include <string>
#include <vector>

class WaveReader
{
    public:
        WaveReader(const std::string &filename, bool &continueFlag);
        virtual ~WaveReader();
        WaveReader(const WaveReader &) = delete;
        WaveReader(WaveReader &&) = delete;
        WaveReader &operator=(const WaveReader &) = delete;
        std::string getFilename() const;
        PCMWaveHeader getHeader() const;
        std::vector<Sample> getSamples(uint32_t quantity, bool &continueFlag);
        bool setSampleOffset(uint32_t offset);
    private:
        std::vector<uint8_t> readData(uint32_t bytesToRead, bool headerBytes, bool &continueFlag);

        std::string filename;
        PCMWaveHeader header;
        uint32_t dataOffset, headerOffset, currentDataOffset;
        int fileDescriptor;
};

#endif // WAVE_READER_HPP
