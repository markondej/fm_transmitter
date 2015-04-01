#ifndef WAVE_READER_H
#define WAVE_READER_H

#include <string>
#include <vector>
#include "pcm_wave_header.h"

class WaveReader
{
    public:
        WaveReader(std::string filename);
        virtual ~WaveReader();
        int checkDataFormat(PCMWaveHeader *header);
        std::vector<unsigned int> *generateFreqDivs(double frequency);
        PCMWaveHeader *getHeader();
    private:
        PCMWaveHeader header;
        std::vector<char> data;
};

#endif // WAVE_READER_H
