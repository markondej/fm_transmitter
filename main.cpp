/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2015, Marcin Kondej
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

#include <iostream>
#include "transmitter.h"
#include <cstdlib>
#include <csignal>

using namespace std;

Transmitter* transmitter = NULL;

void sigIntHandler(int sigNum)
{
    if (transmitter != NULL) {
        cout << "Stopping..." << endl;
        transmitter->stop();
    }
}

int main(int argc, char** argv)
{
    double frequency = 100.0;
    bool loop = false;
    string filename;

    bool showUsage = true;
    for (int i = 1; i < argc; i++) {
        if (string("-f") == argv[i]) {
            if (i < argc - 1) {
                frequency = ::atof(argv[i + 1]);
                i++;
            }
        } else if (string("-r") == argv[i]) {
            loop = true;
        } else {
            if (i == argc - 1) {
                showUsage = false;
                filename = argv[i];
            }
        }
    }
    if (showUsage) {
        cout << "Usage: " << argv[0] << " [-f frequency] [-r] FILE" << endl;
        return 0;
    }

    signal(SIGINT, sigIntHandler);

    try {
        transmitter = Transmitter::getInstance();

        AudioFormat* format = Transmitter::getFormat(filename);
        cout << "Playing: " << ((filename != "-") ? filename : "stdin") << ", "
             << format->sampleRate << " Hz, "
             << format->bitsPerSample << " bits, "
             << ((format->channels > 0x01) ? "stereo" : "mono") << endl;
        delete format;

        transmitter->play(filename, frequency, loop);
    } catch (exception &error) {
        cout << "Error: " << error.what() << endl;
        return 1;
    }

    return 0;
}
