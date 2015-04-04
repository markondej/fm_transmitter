#include "wave_reader.h"
#include <string.h>
#include <exception>
#include <iostream>
#include <fstream>
#include <math.h>

WaveReader::WaveReader(std::string filename)
{
    std::vector<char> buffer;
    unsigned int length;
    int error;

    std::ifstream is(filename.c_str(), std::ifstream::binary);

    if (!is) {
        std::cout << "Error: cannot open '" << filename << "', file does not exist" << std::endl;
        throw std::exception();
    }

    is.seekg(0, is.end);
    length = is.tellg();
    is.seekg(0, is.beg);

    if (length < sizeof(PCMWaveHeader)) {
        std::cout << "Error: data corrupted" << std::endl;
        throw std::exception();
    }

    buffer.resize(length);
    is.read(&buffer[0], length);
    is.close();

    memcpy(&header, &buffer[0], sizeof(PCMWaveHeader));

    if ((error = checkDataFormat(&header)) > 0) {
        std::cout << "Error: unsupported data format (" << error << ")" << std::endl;
        throw std::exception();
    }

    if (length < sizeof(PCMWaveHeader) + header.subchunk2Size) {
        std::cout << "Error: data corrupted" << std::endl;
        throw std::exception();
    }

    data.resize(header.subchunk2Size);
    memcpy(&data[0], &buffer[sizeof(PCMWaveHeader)], header.subchunk2Size);
}

WaveReader::~WaveReader()
{

}

int WaveReader::checkDataFormat(PCMWaveHeader *header)
{
    int error = 0;

    if (std::string(header->chunkID, 4) != std::string("RIFF")) {
        error |= 0x01;
    }
    if (std::string(header->format, 4) != std::string("WAVE")) {
        error |= (0x01 << 1);
    }
    if (std::string(header->subchunk1ID, 4) != std::string("fmt ")) {
        error |= (0x01 << 2);
    }
    if (header->subchunk1Size != 16) {
        error |= (0x01 << 3);
    }
    if (header->audioFormat != WAVE_FORMAT_PCM) {
        error |= (0x01 << 4);
    }
    if (header->byteRate != (header->bitsPerSample >> 3) * header->channels * header->sampleRate) {
        error |= (0x01 << 5);
    }
    if (header->blockAlign != (header->bitsPerSample >> 3) * header->channels) {
        error |= (0x01 << 6);
    }
    if (((header->bitsPerSample >> 3) != 1) && ((header->bitsPerSample >> 3) != 2)) {
        error |= (0x01 << 7);
    }
    if (std::string(header->subchunk2ID, 4) != std::string("data")) {
        error |= (0x01 << 8);
    }

    return error;
}

std::vector<float> *WaveReader::getSamples()
{
    std::vector<float> *samples = new std::vector<float>();
    unsigned char frameSize = header.channels * (header.bitsPerSample >> 3);
    unsigned int framesCount = header.subchunk2Size / frameSize;
    unsigned int frameOffset = 0, dataOffset;



    while (frameOffset < framesCount) {
        dataOffset = frameOffset * frameSize;
        if (header.channels != 1) {
            if (header.bitsPerSample != 8) {
                //samples->push_back((data[dataOffset + 1] + data[dataOffset + 1]) >> 1);
            } else {
                //samples->push_back(((data[dataOffset + 1] + 0x80) + (data[dataOffset + 3] + 0x80)) >> 1);
            }
        } else {
            if (header.bitsPerSample != 8) {
                samples->push_back((signed char)data[dataOffset + 1] / (float)0x80);
            } else {
                samples->push_back(data[dataOffset] / (float)0x80 - 1.0f);
            }
        }
        frameOffset++;
    }
    return samples;
}

PCMWaveHeader *WaveReader::getHeader()
{
    return &header;
}

