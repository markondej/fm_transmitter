/*
    fm_transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2019, Marcin Kondej
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
#include "preemp.hpp"
#include "error_reporter.hpp"
#include "mailbox.h"
#include <bcm_host.h>
#include <sstream>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>

using std::ostringstream;
using std::vector;

#define PERIPHERALS_PHYS_BASE 0x7E000000
#define BCM2835_PERI_VIRT_BASE 0x20000000
#define DMA0_BASE_OFFSET 0x00007000
#define DMA15_BASE_OFFSET 0x00E05000
#define CLK0_BASE_OFFSET 0x00101070
#define PWMCLK_BASE_OFFSET 0x001010A0
#define GPIO_BASE_OFFSET 0x00200000
#define PWM_BASE_OFFSET 0x0020C000
#define TIMER_BASE_OFFSET 0x00003000

#define BCM2837_MEM_FLAG 0x04
#define BCM2835_MEM_FLAG 0x0C
#define PAGE_SIZE 4096

#define BUFFER_TIME 1000000
#define PWM_WRITES_PER_SAMPLE 10
#define PWM_CHANNEL_RANGE 32

struct TimerRegisters {
    uint32_t ctlStatus;
    uint32_t low;
    uint32_t high;
    uint32_t c0;
    uint32_t c1;
    uint32_t c2;
    uint32_t c3;
};

struct ClockRegisters {
    uint32_t ctl;
    uint32_t div;
};

struct PWMRegisters {
    uint32_t ctl;
    uint32_t status;
    uint32_t dmaConf;
    uint32_t reserved0;
    uint32_t chn1Range;
    uint32_t chn1Data;
    uint32_t fifoIn;
    uint32_t reserved1;
    uint32_t chn2Range;
    uint32_t chn2Data;
};

struct DMAControllBlock {
    uint32_t transferInfo;
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t transferLen;
    uint32_t stride;
    uint32_t nextCbAddress;
    uint32_t reserved0;
    uint32_t reserved1;
};

struct DMARegisters {
    uint32_t ctlStatus;
    uint32_t cbAddress;
    uint32_t transferInfo;
    uint32_t srcAddress;
    uint32_t dstAddress;
    uint32_t transferLen;
    uint32_t stride;
    uint32_t nextCbAddress;
    uint32_t debug;
};

bool Transmitter::transmitting = false;
bool Transmitter::clockInitialized = false;
bool Transmitter::preserveCarrier = false;
void *Transmitter::peripherals = NULL;

Transmitter::Transmitter()
    : memSize(0)
{
    int memFd;
    if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw ErrorReporter("Cannot open /dev/mem (permission denied)");
    }

    peripherals = mmap(NULL, bcm_host_get_peripheral_size(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, bcm_host_get_peripheral_address());
    close(memFd);
    if (peripherals == MAP_FAILED) {
        throw ErrorReporter("Cannot obtain access to peripherals (mmap error)");
    }
}

Transmitter::~Transmitter()
{
    munmap(peripherals, bcm_host_get_peripheral_size());
}

Transmitter &Transmitter::getInstance()
{
    static Transmitter instance;
    return instance;
}

bool Transmitter::allocateMemory(uint32_t size)
{
    if (memSize) {
        return false;
    }
    mBoxFd = mbox_open();
    memSize = size;
    if (memSize % PAGE_SIZE) {
        memSize = (memSize / PAGE_SIZE + 1) * PAGE_SIZE;
    }
    memHandle = mem_alloc(mBoxFd, size, PAGE_SIZE, (bcm_host_get_peripheral_address() == BCM2835_PERI_VIRT_BASE) ? BCM2835_MEM_FLAG : BCM2837_MEM_FLAG);
    if (!memHandle) {
        mbox_close(mBoxFd);
        memSize = 0;
        return false;
    }
    memAddress = mem_lock(mBoxFd, memHandle);
    memAllocated = mapmem(memAddress & ~0xC0000000, memSize);
    return true;
}

void Transmitter::freeMemory()
{
    unmapmem(memAllocated, memSize);
    mem_unlock(mBoxFd, memHandle);
    mem_free(mBoxFd, memHandle);

    mbox_close(mBoxFd);
    memSize = 0;
}

uint32_t Transmitter::getMemoryAddress(volatile void *object)
{
    return (memSize) ? memAddress + ((uint32_t)object - (uint32_t)memAllocated) : 0x00000000;
}

uint32_t Transmitter::getPeripheralAddress(volatile void *object) {
    return PERIPHERALS_PHYS_BASE + ((uint32_t)object - (uint32_t)peripherals);
}

void *Transmitter::getPeripheral(uint32_t offset) {
    return (void *)((uint32_t)peripherals + offset);
}

void Transmitter::play(WaveReader &reader, double frequency, double bandwidth, uint8_t dmaChannel, bool preserveCarrierOnExit)
{
    if (transmitting) {
        throw ErrorReporter("Cannot play, transmitter already in use");
    }

    transmitting = true;
    preserveCarrier = preserveCarrierOnExit;

    PCMWaveHeader header = reader.getHeader();
    uint32_t bufferSize = (uint32_t)((uint64_t)header.sampleRate * BUFFER_TIME / 1000000);
    vector<Sample> *samples = reader.getSamples(bufferSize, transmitting);
    if (samples == NULL) {
        return;
    }
    bool eof = samples->size() < bufferSize;

    uint32_t clockDivisor = (uint32_t)round((500 << 12) / frequency);
    uint32_t divisorRange = clockDivisor - (uint32_t)round((500 << 12) / (frequency + 0.0005 * bandwidth));
    bool isError = false;
    string errorMessage;

    if (dmaChannel != 0xFF) {
        if (dmaChannel > 15) {
            delete samples;
            throw ErrorReporter("DMA channel number out of range (0 - 15)");
        }

        if (!allocateMemory(sizeof(uint32_t) * (bufferSize) + 1) + sizeof(DMAControllBlock) * (2 * bufferSize))) {
            delete samples;
            throw ErrorReporter("Cannot allocate memory");
        }

        volatile ClockRegisters *clk0 = (ClockRegisters *)getPeripheral(CLK0_BASE_OFFSET);
        volatile uint32_t *gpio = (uint32_t *)getPeripheral(GPIO_BASE_OFFSET);
        if (!clockInitialized) {
            clk0->ctl = (0x5A << 24) | 0x06;
            usleep(1000);
            clk0->div = (0x5A << 24) | clockDivisor;
            clk0->ctl = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;
            *gpio = (*gpio & 0xFFFF8FFF) | (0x01 << 14);
            clockInitialized = true;
        }

        volatile ClockRegisters *pwmClk = (ClockRegisters *)getPeripheral(PWMCLK_BASE_OFFSET);
        float pwmClkFreq = PWM_WRITES_PER_SAMPLE * PWM_CHANNEL_RANGE * header.sampleRate / 1000000;
        pwmClk->ctl = (0x5A << 24) | 0x06;
        usleep(1000);
        pwmClk->div = (0x5A << 24) | (uint32_t)((500 << 12) / pwmClkFreq);
        pwmClk->ctl = (0x5A << 24) | (0x01 << 4) | 0x06;

        volatile PWMRegisters *pwm = (PWMRegisters *)getPeripheral(PWM_BASE_OFFSET);
        pwm->ctl = 0x00;
        usleep(1000);
        pwm->status = 0x01FC;
        pwm->ctl = (0x01 << 6);
        usleep(1000);
        pwm->chn1Range = PWM_CHANNEL_RANGE;
        pwm->dmaConf = (0x01 << 31) | 0x0707;
        pwm->ctl = (0x01 << 5) | (0x01 << 2) | 0x01;

        float value;
        uint32_t i, cbIndex = 0;
#ifndef NO_PREEMP
        PreEmp preEmp(header.sampleRate);
#endif

        volatile DMAControllBlock *dmaCb = (DMAControllBlock *)memAllocated;
        volatile unsigned *clkDiv = (uint32_t *)((uint32_t)dmaCb + 2 * sizeof(DMAControllBlock) * bufferSize);
        volatile unsigned *pwmFifoData = (uint32_t *)((uint32_t)clkDiv + sizeof(uint32_t) * bufferSize);
        for (i = 0; i < bufferSize; i++) {
            value = (*samples)[i].getMonoValue();
#ifndef NO_PREEMP
            value = preEmp.filter(value);
#endif
            clkDiv[i] = (0x5A << 24) | (clockDivisor - (int32_t)round(value * divisorRange));
            dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x01 << 3);
            dmaCb[cbIndex].srcAddress = getMemoryAddress(&clkDiv[i]);
            dmaCb[cbIndex].dstAddress = getPeripheralAddress(&clk0->div);
            dmaCb[cbIndex].transferLen = sizeof(uint32_t);
            dmaCb[cbIndex].stride = 0;
            dmaCb[cbIndex].nextCbAddress = getMemoryAddress(&dmaCb[cbIndex + 1]);
            cbIndex++;

            dmaCb[cbIndex].transferInfo = (0x01 << 26) | (0x05 << 16) | (0x01 << 6) | (0x01 << 3);
            dmaCb[cbIndex].srcAddress = getMemoryAddress(pwmFifoData);
            dmaCb[cbIndex].dstAddress = getPeripheralAddress(&pwm->fifoIn);
            dmaCb[cbIndex].transferLen = sizeof(uint32_t) * PWM_WRITES_PER_SAMPLE;
            dmaCb[cbIndex].stride = 0;
            dmaCb[cbIndex].nextCbAddress = getMemoryAddress((i < bufferSize - 1) ? &dmaCb[cbIndex + 1] : dmaCb);
            cbIndex++;
        }
        *pwmFifoData = 0x00;        
        delete samples;

        volatile DMARegisters *dma = (DMARegisters *)getPeripheral((dmaChannel < 15) ? DMA0_BASE_OFFSET + dmaChannel * 0x100 : DMA15_BASE_OFFSET);
        dma->ctlStatus = (0x01 << 31);
        usleep(1000);
        dma->ctlStatus = (0x01 << 2) | (0x01 << 1);
        dma->cbAddress = getMemoryAddress(dmaCb);
        dma->ctlStatus = (0xFF << 16) | 0x01;

        usleep(BUFFER_TIME / 4);

        try {
            while (!eof && transmitting) {
                samples = reader.getSamples(bufferSize, transmitting);
                if (samples == NULL) {
                    break;
                }
                eof = samples->size() < bufferSize;
                cbIndex = 0;
                for (i = 0; i < samples->size(); i++) {
                    value = (*samples)[i].getMonoValue();
#ifndef NO_PREEMP
                    value = preEmp.filter(value);
#endif
                    while (i == ((dma->cbAddress - getMemoryAddress(dmaCb)) / (2 * sizeof(DMAControllBlock)))) {
                        usleep(1);
                    }
                    clkDiv[i] = (0x5A << 24) | (clockDivisor - (int32_t)round(value * divisorRange));
                    cbIndex += 2;
                }
               delete samples;
            }
        } catch (ErrorReporter &error) {
            preserveCarrier = false;
            errorMessage = error.what();
            isError = true;
        }

        cbIndex -= 2;
        dmaCb[cbIndex].nextCbAddress = 0x00;
        while (dma->cbAddress != 0x00) {
            usleep(1);
        }

        dma->ctlStatus = (0x01 << 31);
        pwm->ctl = 0x00;

        freeMemory();
        transmitting = false;

        if (!preserveCarrier) {
            clk0->ctl = (0x5A << 24) | 0x06;
        }
    } else {
        uint32_t sampleOffset = 0;
        vector<Sample> *buffer = samples;

        void *transmitterParams[5] = {
            (void *)&buffer,
            (void *)&sampleOffset,
            (void *)&clockDivisor,
            (void *)&divisorRange,
            (void *)&header.sampleRate
        };

        pthread_t thread;
        int returnCode = pthread_create(&thread, NULL, &Transmitter::transmit, (void *)transmitterParams);
        if (returnCode) {
            delete samples;
            ostringstream oss;
            oss << "Cannot create transmitter thread (code: " << returnCode << ")";
            throw ErrorReporter(oss.str());
        }

        usleep(BUFFER_TIME / 2);

        try {
            while (!eof && transmitting) {
                if (buffer == NULL) {
                    if (!reader.setSampleOffset(sampleOffset + bufferSize)) {
                        break;
                    }
                    samples = reader.getSamples(bufferSize, transmitting);
                    if (samples == NULL) {
                        break;
                    }
                    eof = samples->size() < bufferSize;
                    buffer = samples;
                }
                usleep(BUFFER_TIME / 2);
            }
        }
        catch (ErrorReporter &error) {
            preserveCarrier = false;
            errorMessage = error.what();
            isError = true;
        }
        transmitting = false;
        pthread_join(thread, NULL);
        if (buffer != NULL) {
            delete buffer;
        }
    }
    if (isError) {
        throw ErrorReporter(errorMessage);
    }
}

void *Transmitter::transmit(void *params)
{
    vector<Sample> **buffer = (vector<Sample> **)((void **)params)[0];
    uint32_t *sampleOffset = (uint32_t *)((void **)params)[1];
    uint32_t *clockDivisor = (uint32_t *)((void **)params)[2];
    uint32_t *divisorRange = (uint32_t *)((void **)params)[3];
    uint32_t *sampleRate = (uint32_t *)((void **)params)[4];

    uint32_t offset, length, prevOffset;
    vector<Sample> *samples = NULL;
    uint64_t start;
    float value;

#ifndef NO_PREEMP
	PreEmp preEmp(*sampleRate);
#endif

    volatile ClockRegisters *clk0 = (ClockRegisters *)getPeripheral(CLK0_BASE_OFFSET);
    volatile uint32_t *gpio = (uint32_t *)getPeripheral(GPIO_BASE_OFFSET);
    if (!clockInitialized) {
        clk0->ctl = (0x5A << 24) | 0x06;
        usleep(1000);
        clk0->div = (0x5A << 24) | *clockDivisor;
        clk0->ctl = (0x5A << 24) | (0x01 << 9) | (0x01 << 4) | 0x06;
        *gpio = (*gpio & 0xFFFF8FFF) | (0x01 << 14);
        clockInitialized = true;
    }

    volatile TimerRegisters *timer = (TimerRegisters *)getPeripheral(TIMER_BASE_OFFSET);
    uint64_t current = *(uint64_t *)&timer->low;
    uint64_t playbackStart = current;

    while (transmitting) {
        start = current;

        while ((*buffer == NULL) && transmitting) {
            usleep(1);
            current = *(uint64_t *)&timer->low;
        }
        if (!transmitting) {
            break;
        }

        samples = *buffer;
        length = samples->size();
        *buffer = NULL;

        *sampleOffset = (current - playbackStart) * (*sampleRate) / 1000000;
        offset = (current - start) * (*sampleRate) / 1000000;

        while (true) {
            if (offset >= length) {
                break;
            }
            prevOffset = offset;
            value = (*samples)[offset].getMonoValue();
#ifndef NO_PREEMP
            value = preEmp.filter(value);
#endif
            clk0->div = (0x5A << 24) | (*clockDivisor - (int32_t)round(value * (*divisorRange)));
            while (offset == prevOffset) {
                usleep(1); // asm("nop"); 
                current = *(uint64_t *)&timer->low;
                offset = (current - start) * (*sampleRate) / 1000000;
            }
        }
        delete samples;
    }

    if (!preserveCarrier) {
        clk0->ctl = (0x5A << 24) | 0x06;
    }

    return NULL;
}

void Transmitter::stop()
{
    preserveCarrier = false;
    transmitting = false;
}
