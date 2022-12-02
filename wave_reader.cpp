#include "wave_reader.hpp"
#include <stdexcept>
#include <cstring>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>

WaveReader::WaveReader(const std::string &filename, bool &stop) :
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
        ReadData(sizeof(WaveHeader::chunkID) + sizeof(WaveHeader::chunkSize) + sizeof(WaveHeader::format), true, stop);
        if ((std::string(reinterpret_cast<char *>(header.chunkID), 4) != std::string("RIFF")) || (std::string(reinterpret_cast<char *>(header.format), 4) != std::string("WAVE"))) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", WAVE file expected"));
        }

        ReadData(sizeof(WaveHeader::subchunk1ID) + sizeof(WaveHeader::subchunk1Size), true, stop);
        unsigned subchunk1MinSize = sizeof(WaveHeader::audioFormat) + sizeof(WaveHeader::channels) +
            sizeof(WaveHeader::sampleRate) + sizeof(WaveHeader::byteRate) + sizeof(WaveHeader::blockAlign) +
            sizeof(WaveHeader::bitsPerSample);
        if ((std::string(reinterpret_cast<char *>(header.subchunk1ID), 4) != std::string("fmt ")) || (header.subchunk1Size < subchunk1MinSize)) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", data corrupted"));
        }

        ReadData(header.subchunk1Size, true, stop);
        if ((header.audioFormat != WAVE_FORMAT_PCM) ||
            (header.byteRate != (header.bitsPerSample >> 3) * header.channels * header.sampleRate) ||
            (header.blockAlign != (header.bitsPerSample >> 3) * header.channels) ||
            (((header.bitsPerSample >> 3) != 1) && ((header.bitsPerSample >> 3) != 2))) {
            throw std::runtime_error(std::string("Error while opening ") + GetFilename() + std::string(", unsupported WAVE format"));
        }

        ReadData(sizeof(WaveHeader::subchunk2ID) + sizeof(WaveHeader::subchunk2Size), true, stop);
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

std::vector<std::vector<int16_t>> WaveReader::GetFrames(std::size_t quantity, bool &stop) {
    uint16_t bytesPerSample = (header.bitsPerSample >> 3);
    uint16_t bytesPerFrame = bytesPerSample * header.channels;
    std::size_t bytesToRead = quantity * bytesPerFrame;
    std::size_t bytesLeft = header.subchunk2Size - currentDataOffset;
    if (bytesToRead > bytesLeft) {
        bytesToRead = bytesLeft - bytesLeft % bytesPerFrame;
        quantity = bytesToRead / bytesPerFrame;
    }

    std::vector<uint8_t> data = std::move(ReadData(bytesToRead, false, stop));
    if (data.size() < bytesToRead) {
        quantity = data.size() / bytesPerFrame;
    }

    std::vector<std::vector<int16_t>> frames;
    frames.reserve(quantity);
    for (std::size_t i = 0; i < quantity; i++) {
        std::vector<int16_t> channels(header.channels, 0x0000);
        for (int16_t j = 0; j < header.channels; j++) {
            switch (bytesPerSample) {
            case 2:
                channels[j] = (data[i * bytesPerFrame + j * bytesPerSample + 1] << 8) | data[i * bytesPerFrame + j * bytesPerSample];
                break;
            case 1:
                channels[j] = (static_cast<int16_t>(data[i * bytesPerFrame]) - 0x80) << 8;
                break;
            }
        }
        frames.push_back(channels);
    }
    return frames;
}

bool WaveReader::SetFrameOffset(std::size_t offset) {
    if (fileDescriptor != STDIN_FILENO) {
        currentDataOffset = offset * (header.bitsPerSample >> 3) * header.channels;
        if (lseek(fileDescriptor, dataOffset + currentDataOffset, SEEK_SET) == -1) {
            return false;
        }
    }
    return true;
}

std::vector<uint8_t> WaveReader::ReadData(std::size_t bytesToRead, bool headerBytes, bool &stop)
{
    std::size_t bytesRead = 0;
    std::vector<uint8_t> data;
    data.resize(bytesToRead);
    while ((bytesRead < bytesToRead) && !stop) {
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
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
    }

    if (headerBytes) {
        if (stop) {
            throw std::runtime_error("Cannot obtain header, program interrupted");
        }
        std::memcpy(&(reinterpret_cast<uint8_t *>(&header))[headerOffset], data.data(), bytesRead);
        headerOffset += bytesRead;
    } else {
        if (stop) {
            data.resize(bytesRead);
        }
        currentDataOffset += bytesRead;
    }

    return data;
}
