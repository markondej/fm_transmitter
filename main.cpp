#include <stdlib.h>
#include <iostream>
#include "wave_reader.h"
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
        WaveReader *reader = new WaveReader(filename);
        Transmitter *transmitter = new Transmitter(frequency);
        PCMWaveHeader *header = reader->getHeader();
        std::vector<float> *samples = reader->getSamples();
        transmitter->transmit(samples, header->sampleRate);
        delete transmitter;
        delete samples;
        delete reader;
    } catch (exception &e) {
        return 1;
    }
    return 0;
}
