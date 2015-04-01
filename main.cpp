#include <stdlib.h>
#include <iostream>
#include "pcm_wave_reader.h"
#include "transmitter.h"

using namespace std;

int main(int argc, char **argv)
{
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " [FILE] [frequency]" << endl;
        return 0;
    }

    string filename = argv[1];
    double frequency = (argc < 3) ? 100.0 : (::atof(argv[2]));
    try {
        PCMWaveReader *reader = new PCMWaveReader(filename);
        Transmitter *transmitter = new Transmitter(frequency);
        PCMWaveHeader *header = reader->getHeader();
        std::vector<unsigned int> *freqDivs = reader->generateFreqDivs(frequency);
        transmitter->transmit(freqDivs, header->sampleRate);
        delete freqDivs;
        delete transmitter;
        delete reader;
    } catch (exception &e) {
        return 1;
    }
    return 0;
}
