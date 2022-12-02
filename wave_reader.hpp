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
        std::vector<std::vector<int16_t>> GetFrames(std::size_t quantity, bool &stop);
        bool SetFrameOffset(std::size_t offset);
    private:
        std::vector<uint8_t> ReadData(std::size_t bytesToRead, bool headerBytes, bool &stop);

        std::string filename;
        WaveHeader header;
        std::size_t dataOffset, headerOffset, currentDataOffset;
        int fileDescriptor;
};
