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
    int sum = 0;
    int16_t *channelValues = new int16_t[channels];
    for (unsigned i = 0; i < channels; i++) {
        switch (bitsPerChannel >> 3) {
        case 2:
            channelValues[i] = (data[((i + 1) << 1) - 1] << 8) | data[((i + 1) << 1) - 2];
            break;
        case 1:
            channelValues[i] = (static_cast<int16_t>(data[i]) - 0x80) << 8;
            break;
        }
        sum += channelValues[i];
    }
    value = 2 * sum / (static_cast<float>(USHRT_MAX) * channels);
    delete[] channelValues;
}

float Sample::GetMonoValue() const
{
    return value;
}

WaveReader::WaveReader(const std::string &filename, bool &enable, std::mutex &mtx) :
    filename(filename), headerOffset(0), currentDataOffset(0)
{
    if (!filename.empty()) {
        fileDescriptor = open(filename.c_str(), O_RDONLY);
    } else {
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        fileDescriptor = STDIN_FILENO;
    }

    if (fileDescriptor == -1) {
        throw std::runtime_error(std::string("Cannot open ") + GetFilename() + std::string(", file does not exist"));
    }

    try {
        ReadData(sizeof(WaveHeader::chunkID) + sizeof(WaveHeader::chunkSize) + sizeof(WaveHeader::format), true, enable, mtx);
        if ((std::string(reinterpret_cast<char *>(header.chunkID), 4) != std::string("RIFF")) || (std::string(reinterpret_cast<char *>(header.format), 4) != std::string("WAVE"))) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", WAVE file expected"));
        }

        ReadData(sizeof(WaveHeader::subchunk1ID) + sizeof(WaveHeader::subchunk1Size), true, enable, mtx);
        unsigned subchunk1MinSize = sizeof(WaveHeader::audioFormat) + sizeof(WaveHeader::channels) +
            sizeof(WaveHeader::sampleRate) + sizeof(WaveHeader::byteRate) + sizeof(WaveHeader::blockAlign) +
            sizeof(WaveHeader::bitsPerSample);
        if ((std::string(reinterpret_cast<char *>(header.subchunk1ID), 4) != std::string("fmt ")) || (header.subchunk1Size < subchunk1MinSize)) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", data corrupted"));
        }

        ReadData(header.subchunk1Size, true, enable, mtx);
        if ((header.audioFormat != WAVE_FORMAT_PCM) ||
            (header.byteRate != (header.bitsPerSample >> 3) * header.channels * header.sampleRate) ||
            (header.blockAlign != (header.bitsPerSample >> 3) * header.channels) ||
            (((header.bitsPerSample >> 3) != 1) && ((header.bitsPerSample >> 3) != 2))) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", unsupported WAVE format"));
        }

        ReadData(sizeof(WaveHeader::subchunk2ID) + sizeof(WaveHeader::subchunk2Size), true, enable, mtx);
        if (std::string(reinterpret_cast<char *>(header.subchunk2ID), 4) != std::string("data")) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", data corrupted"));
        }
    } catch (...) {
        if (fileDescriptor != STDIN_FILENO) {
            close(fileDescriptor);
        }
        throw;
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

std::string WaveReader::GetFilename() const
{
    return fileDescriptor != STDIN_FILENO ? filename : "STDIN";
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

    std::vector<uint8_t> data = std::move(ReadData(bytesToRead, false, enable, mtx));
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
    if (fileDescriptor != STDIN_FILENO) {
        currentDataOffset = offset * (header.bitsPerSample >> 3) * header.channels;
        if (lseek(fileDescriptor, dataOffset + currentDataOffset, SEEK_SET) == -1) {
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
    timeval timeout = {
        .tv_sec = 1,
    };
    fd_set fds;
    while (bytesRead < bytesToRead) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (!enable) {
                break;
            }
        }
        int bytes = read(fileDescriptor, &data[bytesRead], bytesToRead - bytesRead);
        if (((bytes == -1) && ((fileDescriptor != STDIN_FILENO) || (errno != EAGAIN))) ||
            ((static_cast<unsigned>(bytes) < bytesToRead) && headerBytes && (fileDescriptor != STDIN_FILENO))) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", data corrupted"));
        }
        if (bytes > 0) {
            bytesRead += bytes;
        }
        if (bytesRead < bytesToRead) {
            if (fileDescriptor != STDIN_FILENO) {
                data.resize(bytesRead);
                break;
            } else {
                FD_ZERO(&fds);
                FD_SET(STDIN_FILENO, &fds);
                select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout);
                if (FD_ISSET(STDIN_FILENO, &fds)) {
                    FD_CLR(STDIN_FILENO, &fds);
                }
            }
        }
    }

    if (headerBytes) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (!enable) {
                throw std::runtime_error("Cannot obtain header, program interrupted");
            }
        }
        std::memcpy(&(reinterpret_cast<uint8_t *>(&header))[headerOffset], data.data(), bytesRead);
        headerOffset += bytesRead;
    } else {
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (!enable) {
                data.resize(bytesRead);
            }
        }
        currentDataOffset += bytesRead;
    }

    return data;
}
