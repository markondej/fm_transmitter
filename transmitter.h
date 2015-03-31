#ifndef TRANSMITTER_H
#define TRANSMITTER_H


class Transmitter
{
    public:
        Transmitter(float frequency);
        virtual ~Transmitter();
    private:
        float frequency;
        volatile unsigned *gpio;
        volatile unsigned *clock;
};

#endif // TRANSMITTER_H
