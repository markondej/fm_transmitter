/*
    FM Transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2020, Marcin Kondej
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

#include "transmitter.hpp"
#include <cstdlib>
#include <csignal>
#include <iostream>
#include <unistd.h>

bool stop = false;
Transmitter *transmitter = nullptr;

void sigIntHandler(int sigNum)
{
    if (transmitter != nullptr) {
        std::cout << "Stopping..." << std::endl;
        transmitter->Stop();
        stop = true;
    }
}

int main(int argc, char** argv)
{
    float frequency = 100.f, bandwidth = 200.f;
    uint16_t dmaChannel = 0;
    bool showUsage = true, loop = false;
    int opt, filesOffset;

    while ((opt = getopt(argc, argv, "rf:d:b:v")) != -1) {
        switch (opt) {
            case 'r':
                loop = true;
                break;
            case 'f':
                frequency = ::atof(optarg);
                break;
            case 'd':
                dmaChannel = ::atof(optarg);
                break;
            case 'b':
                bandwidth = ::atof(optarg);
                break;
            case 'v':
                std::cout << EXECUTABLE << " version: " << VERSION << std::endl;
                return 0;
        }
    }
    if (optind < argc) {
        filesOffset = optind;
        showUsage = false;
    }
    if (showUsage) {
        std::cout << "Usage: " << EXECUTABLE << " [-f <frequency>] [-b <bandwidth>] [-d <dma_channel>] [-r] <file>" << std::endl;
        return 0;
    }

    signal(SIGINT, sigIntHandler);
    signal(SIGTSTP, sigIntHandler);

    auto finally = [&]() {
        delete transmitter;
        transmitter = nullptr;
    };
    try {
        transmitter = new Transmitter();
        std::cout << "Broadcasting at " << frequency << " MHz with "
            << bandwidth << " kHz bandwidth" << std::endl;
        do {
            std::string filename = argv[optind++];
            if ((optind == argc) && loop) {
                optind = filesOffset;
            }
            WaveReader reader(filename != "-" ? filename : std::string(), stop);
            WaveHeader header = reader.GetHeader();
            std::cout << "Playing: " << reader.GetFilename() << ", "
                << header.sampleRate << " Hz, "
                << header.bitsPerSample << " bits, "
                << ((header.channels > 0x01) ? "stereo" : "mono") << std::endl;
            transmitter->Transmit(reader, frequency, bandwidth, dmaChannel, optind < argc);
        } while (!stop && (optind < argc));
    } catch (std::exception &catched) {
        std::cout << "Error: " << catched.what() << std::endl;
        finally();
        return 1;
    }
    finally();

    return 0;
}
