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

#include "wave_reader.hpp"
#include <stdexcept>
#include <cstring>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <climits>

Sample::Sample(uint8_t *data, unsigned channels, unsigned bitsPerChannel)
    : value(0.f)
{
    uint32_t sum = 0;
    for (unsigned i = 0; i < channels; i++) {
        switch (bitsPerChannel >> 3) {
        case 2:
            sum += *reinterpret_cast<int16_t *>(&data[i << 1]);
            break;
        case 1:
            sum += (static_cast<int16_t>(data[i]) - 0x80) << 8;
            break;
        }
    }
    value = sum / (-static_cast<float>(SHRT_MIN) * channels);
}

float Sample::GetMonoValue() const
{
    return value;
}

WaveReader::WaveReader(const std::string &filename, bool &enable, std::mutex &mtx) :
    filename(filename), headerOffset(0), currentDataOffset(0)
{
    if (!filename.empty()) {
        fd = open(filename.c_str(), O_RDONLY);
    } else {
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        fd = STDIN_FILENO;
    }

    if (fd == -1) {
        throw std::runtime_error(std::string("Cannot open ") + GetFilename() + std::string(", file does not exist"));
    }

    try {
        ReadData(sizeof(WaveHeader::chunkID) + sizeof(WaveHeader::chunkSize) + sizeof(WaveHeader::format), true, enable, mtx);
        if ((std::string(reinterpret_cast<char *>(header.chunkID), 4) != std::string("RIFF")) || (std::string(reinterpret_cast<char *>(header.format), 4) != std::string("WAVE"))) {
            throw std::runtime_error("Error while reading " + GetFilename() + ", WAVE file expected");
        }

        ReadData(sizeof(WaveHeader::subchunk1ID) + sizeof(WaveHeader::subchunk1Size), true, enable, mtx);
        unsigned subchunk1MinSize = sizeof(WaveHeader::audioFormat) + sizeof(WaveHeader::channels) +
            sizeof(WaveHeader::sampleRate) + sizeof(WaveHeader::byteRate) + sizeof(WaveHeader::blockAlign) +
            sizeof(WaveHeader::bitsPerSample);
        if ((std::string(reinterpret_cast<char *>(header.subchunk1ID), 4) != std::string("fmt ")) || (header.subchunk1Size < subchunk1MinSize)) {
            throw std::runtime_error("Error while reading " + GetFilename() + ", data corrupted");
        }

        ReadData(header.subchunk1Size, true, enable, mtx);
        if ((header.audioFormat != WAVE_FORMAT_PCM) ||
            (header.byteRate != (header.bitsPerSample >> 3) * header.channels * header.sampleRate) ||
            (header.blockAlign != (header.bitsPerSample >> 3) * header.channels) ||
            (((header.bitsPerSample >> 3) != 1) && ((header.bitsPerSample >> 3) != 2))) {
            throw std::runtime_error("Error while reading " + GetFilename() + ", unsupported WAVE format");
        }

        ReadData(sizeof(WaveHeader::subchunk2ID) + sizeof(WaveHeader::subchunk2Size), true, enable, mtx);
        if (std::string(reinterpret_cast<char *>(header.subchunk2ID), 4) != std::string("data")) {
            throw std::runtime_error("Error while opening " + GetFilename() + ", data corrupted");
        }
    } catch (...) {
        if (fd != STDIN_FILENO) {
            close(fd);
        }
        throw;
    }

    if (fd != STDIN_FILENO) {
        dataOffset = lseek(fd, 0, SEEK_CUR);
    }
}

WaveReader::~WaveReader()
{
    if (fd != STDIN_FILENO) {
        close(fd);
    }
}

std::string WaveReader::GetFilename() const
{
    return fd != STDIN_FILENO ? filename : "STDIN";
}

const WaveHeader &WaveReader::GetHeader() const
{
    return header;
}

std::vector<Sample> WaveReader::GetSamples(unsigned quantity, bool &enable, std::mutex &mtx) {
    unsigned bytesPerSample = (header.bitsPerSample >> 3) * header.channels;
    unsigned bytesToRead = quantity * bytesPerSample;
    unsigned bytesLeft = header.subchunk2Size - currentDataOffset;
    if (bytesToRead > bytesLeft) {
        bytesToRead = bytesLeft - bytesLeft % bytesPerSample;
        quantity = bytesToRead / bytesPerSample;
    }

    std::vector<uint8_t> data = ReadData(bytesToRead, false, enable, mtx);
    if (data.size() < bytesToRead) {
        quantity = data.size() / bytesPerSample;
    }

    std::vector<Sample> samples;
    samples.reserve(quantity);
    for (unsigned i = 0; i < quantity; i++) {
        samples.push_back(Sample(&data[bytesPerSample * i], header.channels, header.bitsPerSample));
    }
    return samples;
}

bool WaveReader::SetSampleOffset(unsigned offset) {
    if (fd != STDIN_FILENO) {
        currentDataOffset = offset * (header.bitsPerSample >> 3) * header.channels;
        if (lseek(fd, dataOffset + currentDataOffset, SEEK_SET) == -1) {
            return false;
        }
    }
    return true;
}

std::vector<uint8_t> WaveReader::ReadData(unsigned bytesToRead, bool headerBytes, bool &enable, std::mutex &mtx)
{
    unsigned bytesRead = 0;
    std::vector<uint8_t> data;
    data.resize(bytesToRead);

    while (bytesRead < bytesToRead) {
        std::unique_lock<std::mutex> unique(mtx);
        if (!enable) {
            if (headerBytes) {
                throw std::runtime_error("Failed to read header, operation aborted");
            }
            data.resize(bytesRead);
            break;
        }
        unique.unlock();
        int bytes = read(fd, &data[bytesRead], bytesToRead - bytesRead);
        if ((bytes == -1) && (errno != EAGAIN) && (errno != EINTR)) {
            throw std::runtime_error("Error while reading " + GetFilename() + ", operation failed");
        }
        if (bytes > 0) {
            bytesRead += bytes;
        }
        if (bytesRead < bytesToRead) {
            if (fd != STDIN_FILENO) {
                data.resize(bytesRead);
                break;
            } else {
                timeval timeout = { .tv_sec = 1, };

                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(STDIN_FILENO, &fds);
                select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout);
                if (!FD_ISSET(STDIN_FILENO, &fds)) {
                    data.resize(bytesRead);
                    break;
                }
            }
        }
    }

    if (headerBytes) {
        if (bytesRead < bytesToRead) {
            throw std::runtime_error("Failed to read header, data corrupted");
        }
        std::memcpy(&(reinterpret_cast<uint8_t *>(&header))[headerOffset], data.data(), bytesRead);
        headerOffset += bytesRead;
    } else {
        currentDataOffset += bytesRead;
    }

    return data;
}
