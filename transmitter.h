#ifndef TRANSMITTER_H
#define TRANSMITTER_H


class Transmitter
{
    public:
        Transmitter();
        virtual ~Transmitter();
    private:
        volatile unsigned *gpio;
        volatile unsigned *clock;
};

#endif // TRANSMITTER_H
