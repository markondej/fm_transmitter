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

#include "wave_reader.hpp"
#include <stdexcept>
#include <cstring>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>

WaveReader::WaveReader(const std::string &filename, bool &continueFlag) :
    filename(filename), headerOffset(0), currentDataOffset(0)
{
    if (!filename.empty()) {
        fileDescriptor = open(filename.c_str(), O_RDONLY);
    } else {
        fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
        fileDescriptor = STDIN_FILENO;
    }

    if (fileDescriptor == -1) {
        throw std::runtime_error(std::string("Cannot open ") + getFilename() + std::string(", file does not exist"));
    }

    try {
        readData(sizeof(PCMWaveHeader::chunkID) + sizeof(PCMWaveHeader::chunkSize) + sizeof(PCMWaveHeader::format), true, continueFlag);
        if ((std::string(reinterpret_cast<char *>(header.chunkID), 4) != std::string("RIFF")) || (std::string(reinterpret_cast<char *>(header.format), 4) != std::string("WAVE"))) {
            throw std::runtime_error(std::string("Error while opening ") + getFilename() + std::string(", WAVE file expected"));
        }

        readData(sizeof(PCMWaveHeader::subchunk1ID) + sizeof(PCMWaveHeader::subchunk1Size), true, continueFlag);
        uint32_t subchunk1MinSize = sizeof(PCMWaveHeader::audioFormat) + sizeof(PCMWaveHeader::channels) +
            sizeof(PCMWaveHeader::sampleRate) + sizeof(PCMWaveHeader::byteRate) + sizeof(PCMWaveHeader::blockAlign) +
            sizeof(PCMWaveHeader::bitsPerSample);
        if ((std::string(reinterpret_cast<char *>(header.subchunk1ID), 4) != std::string("fmt ")) || (header.subchunk1Size < subchunk1MinSize)) {
            throw std::runtime_error(std::string("Error while opening ") + getFilename() + std::string(", data corrupted"));
        }

        readData(header.subchunk1Size, true, continueFlag);
        if ((header.audioFormat != WAVE_FORMAT_PCM) ||
            (header.byteRate != (header.bitsPerSample >> 3) * header.channels * header.sampleRate) ||
            (header.blockAlign != (header.bitsPerSample >> 3) * header.channels) ||
            (((header.bitsPerSample >> 3) != 1) && ((header.bitsPerSample >> 3) != 2))) {
            throw std::runtime_error(std::string("Error while opening ") + getFilename() + std::string(", unsupported WAVE format"));
        }

        readData(sizeof(PCMWaveHeader::subchunk2ID) + sizeof(PCMWaveHeader::subchunk2Size), true, continueFlag);
        if (std::string(reinterpret_cast<char *>(header.subchunk2ID), 4) != std::string("data")) {
            throw std::runtime_error(std::string("Error while opening ") + getFilename() + std::string(", data corrupted"));
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

std::vector<uint8_t> WaveReader::readData(uint32_t bytesToRead, bool headerBytes, bool &continueFlag)
{
    uint32_t bytesRead = 0;
    std::vector<uint8_t> data;
    data.resize(bytesToRead);
    while ((bytesRead < bytesToRead) && continueFlag) {
        int bytes = read(fileDescriptor, &data[bytesRead], bytesToRead - bytesRead);
        if (((bytes == -1) && ((fileDescriptor != STDIN_FILENO) || (errno != EAGAIN))) ||
            ((static_cast<uint32_t>(bytes) < bytesToRead) && headerBytes && (fileDescriptor != STDIN_FILENO))) {
            throw std::runtime_error(std::string("Error while opening ") + getFilename() + std::string(", data corrupted"));
        }
        if (bytes > 0) {
            bytesRead += bytes;
        }
        if (bytesRead < bytesToRead) {
            if (fileDescriptor != STDIN_FILENO) {
                data.resize(bytes);
                break;
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
    }

    if (headerBytes) {
        if (!continueFlag) {
            throw std::runtime_error("Cannot obtain header, program interrupted");
        }
        std::memcpy(&(reinterpret_cast<uint8_t *>(&header))[headerOffset], data.data(), bytesRead);
        headerOffset += bytesRead;
    } else {
        currentDataOffset += bytesRead;
    }

    return data;
}

std::vector<Sample> WaveReader::getSamples(uint32_t quantity, bool &continueFlag) {
    uint32_t bytesPerSample = (header.bitsPerSample >> 3) * header.channels;
    uint32_t bytesToRead = quantity * bytesPerSample;
    uint32_t bytesLeft = header.subchunk2Size - currentDataOffset;
    if (bytesToRead > bytesLeft) {
        bytesToRead = bytesLeft - bytesLeft % bytesPerSample;
        quantity = bytesToRead / bytesPerSample;
    }

    std::vector<uint8_t> data = std::move(readData(bytesToRead, false, continueFlag));
    if (data.size() < bytesToRead) {
        quantity = data.size() / bytesPerSample;
    }

    std::vector<Sample> samples;
    for (uint32_t i = 0; i < quantity; i++) {
        samples.push_back(Sample(&data[bytesPerSample * i], header.channels, header.bitsPerSample));
    }
    return samples;
}

bool WaveReader::setSampleOffset(uint32_t offset) {
    if (fileDescriptor != STDIN_FILENO) {
        currentDataOffset = offset * (header.bitsPerSample >> 3) * header.channels;
        if (lseek(fileDescriptor, dataOffset + currentDataOffset, SEEK_SET) == -1) {
            return false;
        }
    }
    return true;
}

std::string WaveReader::getFilename() const
{
    return fileDescriptor != STDIN_FILENO ? filename : "STDIN";
}

PCMWaveHeader WaveReader::getHeader() const
{
    return header;
}
