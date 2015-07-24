#include "stdin_reader.h"

StdinReader::StdinReader()
{
    //ctor
}

StdinReader::~StdinReader()
{
    //dtor
}

vector<float> *StdinReader::getFrames(unsigned int count)
{
    return new vector<float>();
}

AudioFormat *StdinReader::getFormat()
{
    AudioFormat *format = new AudioFormat;
    return format;
}
