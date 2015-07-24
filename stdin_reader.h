#ifndef STDIN_READER_H
#define STDIN_READER_H

#include <vector>
#include "audio_format.h"

using std::vector;

class StdinReader
{
    public:
        StdinReader();
        virtual ~StdinReader();

        vector<float> *getFrames(unsigned int count);
        AudioFormat *getFormat();
};

#endif // STDIN_READER_H
