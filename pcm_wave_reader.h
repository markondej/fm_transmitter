#ifndef PCM_WAVE_READER_H
#define PCM_WAVE_READER_H

#include <string>
#include <vector>
#include "pcm_wave_header.h"

class PCMWaveReader
{
    public:
        PCMWaveReader(std::string filename);
        virtual ~PCMWaveReader();
        int checkDataFormat(PCMWaveHeader *header);
        std::vector<unsigned int> *generateFreqDivs(double frequency);
        PCMWaveHeader *getHeader();
    private:
        PCMWaveHeader header;
        std::vector<char> data;
};

#endif // PCM_WAVE_READER_H
