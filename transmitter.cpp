/*
    FM Transmitter - use Raspberry Pi as FM transmitter

    Copyright (c) 2021, Marcin Kondej
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
#include "mailbox.h"
#include <bcm_host.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <sys/mman.h>

#define PERIPHERALS_PHYS_BASE 0x7e000000
#define BCM2835_PERI_VIRT_BASE 0x20000000
#define BCM2711_PERI_VIRT_BASE 0xfe000000

#define BCM2835_MEM_FLAG 0x0c
#define BCM2711_MEM_FLAG 0x04

#define BCM2835_PLLD_FREQ 500
#define BCM2711_PLLD_FREQ 750

#define GPIO_BASE_OFFSET 0x00200000
#define TIMER_BASE_OFFSET 0x00003000

#define CLK0_BASE_OFFSET 0x00101070
#define CLK1_BASE_OFFSET 0x00101078
#define CLK_PASSWORD (0x5a << 24)
#define CLK_CTL_SRC_PLLA 0x04
#define CLK_CTL_SRC_PLLC 0x05
#define CLK_CTL_SRC_PLLD 0x06
#define CLK_CTL_ENAB (0x01 << 4)
#define CLK_CTL_MASH(x) ((x & 0x03) << 9)

#define PWMCLK_BASE_OFFSET 0x001010a0
#define PWM_BASE_OFFSET 0x0020c000
#define PWM_CHANNEL_RANGE 32
#define PWM_WRITES_PER_SAMPLE 10
#define PWM_CTL_CLRF1 (0x01 << 6)
#define PWM_CTL_USEF1 (0x01 << 5)
#define PWM_CTL_RPTL1 (0x01 << 2)
#define PWM_CTL_MODE1 (0x01 << 1)
#define PWM_CTL_PWEN1 0x01
#define PWM_STA_BERR (0x01 << 8)
#define PWM_STA_GAPO4 (0x01 << 7)
#define PWM_STA_GAPO3 (0x01 << 6)
#define PWM_STA_GAPO2 (0x01 << 5)
#define PWM_STA_GAPO1 (0x01 << 4)
#define PWM_STA_RERR1 (0x01 << 3)
#define PWM_STA_WERR1 (0x01 << 2)
#define PWM_STA_EMPT1 (0x01 << 1)
#define PWM_STA_FULL1 0x01
#define PWM_DMAC_ENAB (0x01 << 31)
#define PWM_DMAC_PANIC(x) ((x & 0x0f) << 8)
#define PWM_DMAC_DREQ(x) (x & 0x0f)

#define DMA0_BASE_OFFSET 0x00007000
#define DMA15_BASE_OFFSET 0x00e05000
#define DMA_CS_RESET (0x01 << 31)
#define DMA_CS_PANIC_PRIORITY(x) ((x & 0x0f) << 20)
#define DMA_CS_PRIORITY(x) ((x & 0x0f) << 16)
#define DMA_CS_INT (0x01 << 2)
#define DMA_CS_END (0x01 << 1)
#define DMA_CS_ACTIVE 0x01
#define DMA_TI_NO_WIDE_BURST (0x01 << 26)
#define DMA_TI_PERMAP(x) ((x & 0x0f) << 16)
#define DMA_TI_DEST_DREQ (0x01 << 6)
#define DMA_TI_WAIT_RESP (0x01 << 3)

#define BUFFER_TIME 1000000
#define PAGE_SIZE 4096

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

class Peripherals
{
    public:
        virtual ~Peripherals() {
            munmap(peripherals, GetSize());
        }
        Peripherals(const Peripherals &) = delete;
        Peripherals(Peripherals &&) = delete;
        Peripherals &operator=(const Peripherals &) = delete;
        static Peripherals &GetInstance() {
            static Peripherals instance;
            return instance;
        }
        inline uintptr_t GetPhysicalAddress(volatile void *object) const {
            return PERIPHERALS_PHYS_BASE + (reinterpret_cast<uintptr_t>(object) - reinterpret_cast<uintptr_t>(peripherals));
        }
        inline uintptr_t GetVirtualAddress(uintptr_t offset) const {
            return reinterpret_cast<uintptr_t>(peripherals) + offset;
        }
        inline static uintptr_t GetVirtualBaseAddress() {
            return (bcm_host_get_peripheral_size() == BCM2711_PERI_VIRT_BASE) ? BCM2711_PERI_VIRT_BASE : bcm_host_get_peripheral_address();
        }
        inline static float GetClockFrequency() {
            return (Peripherals::GetVirtualBaseAddress() == BCM2711_PERI_VIRT_BASE) ? BCM2711_PLLD_FREQ : BCM2835_PLLD_FREQ;
        }
    private:
        Peripherals() {
            int memFd;
            if ((memFd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
                throw std::runtime_error("Cannot open /dev/mem file (permission denied)");
            }

            peripherals = mmap(nullptr, GetSize(), PROT_READ | PROT_WRITE, MAP_SHARED, memFd, GetVirtualBaseAddress());
            close(memFd);
            if (peripherals == MAP_FAILED) {
                throw std::runtime_error("Cannot obtain access to peripherals (mmap error)");
            }
        }
        unsigned GetSize() {
            unsigned size = bcm_host_get_peripheral_size();
            if (size == BCM2711_PERI_VIRT_BASE) {
                size = 0x01000000;
            }
            return size;
        }

        void *peripherals;
};

class AllocatedMemory
{
    public:
        AllocatedMemory() = delete;
        AllocatedMemory(unsigned size) {
            mBoxFd = mbox_open();
            memSize = size;
            if (memSize % PAGE_SIZE) {
                memSize = (memSize / PAGE_SIZE + 1) * PAGE_SIZE;
            }
            memHandle = mem_alloc(mBoxFd, size, PAGE_SIZE, (Peripherals::GetVirtualBaseAddress() == BCM2835_PERI_VIRT_BASE) ? BCM2835_MEM_FLAG : BCM2711_MEM_FLAG);
            if (!memHandle) {
                mbox_close(mBoxFd);
                memSize = 0;
                throw std::runtime_error("Cannot allocate memory (" + std::to_string(size) + " bytes)");
            }
            memAddress = mem_lock(mBoxFd, memHandle);
            memAllocated = mapmem(memAddress & ~0xc0000000, memSize);
        }
        virtual ~AllocatedMemory() {
            unmapmem(memAllocated, memSize);
            mem_unlock(mBoxFd, memHandle);
            mem_free(mBoxFd, memHandle);
            mbox_close(mBoxFd);
            memSize = 0;
        }
        AllocatedMemory(const AllocatedMemory &) = delete;
        AllocatedMemory(AllocatedMemory &&) = delete;
        AllocatedMemory &operator=(const AllocatedMemory &) = delete;
        inline uintptr_t GetPhysicalAddress(volatile void *object) const {
            return (memSize) ? memAddress + (reinterpret_cast<uintptr_t>(object) - reinterpret_cast<uintptr_t>(memAllocated)) : 0x00000000;
        }
        inline uintptr_t GetBaseAddress() const {
            return reinterpret_cast<uintptr_t>(memAllocated);
        }
    private:
        unsigned memSize, memHandle;
        uintptr_t memAddress;
        void *memAllocated;
        int mBoxFd;
};

class Device
{
    public:
        Device() {
            peripherals = &Peripherals::GetInstance();
        }
        Device(const Device &) = delete;
        Device(Device &&) = delete;
        Device &operator=(const Device &) = delete;
    protected:
        Peripherals *peripherals;
};

class ClockDevice : public Device
{
    public:
        ClockDevice() = delete;
        ClockDevice(uintptr_t address, unsigned divisor) {
            clock = reinterpret_cast<ClockRegisters *>(peripherals->GetVirtualAddress(address));
            clock->ctl = CLK_PASSWORD | CLK_CTL_SRC_PLLD;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            clock->div = CLK_PASSWORD | (0xffffff & divisor);
            clock->ctl = CLK_PASSWORD | CLK_CTL_MASH(0x1) | CLK_CTL_ENAB | CLK_CTL_SRC_PLLD;        }
        virtual ~ClockDevice() {
            clock->ctl = CLK_PASSWORD | CLK_CTL_SRC_PLLD;
        }
    protected:
        volatile ClockRegisters *clock;
};

class ClockOutput : public ClockDevice
{
    public:
        ClockOutput() = delete;
#ifndef GPIO21
        ClockOutput(unsigned divisor) : ClockDevice(CLK0_BASE_OFFSET, divisor) {
            output = reinterpret_cast<uint32_t *>(peripherals->GetVirtualAddress(GPIO_BASE_OFFSET));
            *output = (*output & 0xffff8fff) | (0x04 << 12);
#else
        ClockOutput(unsigned divisor) : ClockDevice(CLK1_BASE_OFFSET, divisor) {
            output = reinterpret_cast<uint32_t *>(peripherals->GetVirtualAddress(GPIO_BASE_OFFSET + 0x08));
            *output = (*output & 0xffffffc7) | (0x02 << 3);
#endif
        }
        virtual ~ClockOutput() {
#ifndef GPIO21
            *output = (*output & 0xffff8fff) | (0x01 << 12);
#else
            *output = (*output & 0xffffffc7) | (0x02 << 3);
#endif
        }
        inline void SetDivisor(unsigned divisor) {
            clock->div = CLK_PASSWORD | (0xffffff & divisor);
        }
        inline volatile uint32_t &GetDivisor() {
            return clock->div;
        }
    private:
        volatile uint32_t *output;
};

class PWMController : public ClockDevice
{
    public:
        PWMController() = delete;
        PWMController(unsigned sampleRate) : ClockDevice(PWMCLK_BASE_OFFSET, static_cast<unsigned>(Peripherals::GetClockFrequency() * 1000000.f * (0x01 << 12) / (PWM_WRITES_PER_SAMPLE * PWM_CHANNEL_RANGE * sampleRate))) {
            pwm = reinterpret_cast<PWMRegisters *>(peripherals->GetVirtualAddress(PWM_BASE_OFFSET));
            pwm->ctl = 0x00000000;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            pwm->status = PWM_STA_BERR | PWM_STA_GAPO1 | PWM_STA_RERR1 | PWM_STA_WERR1;
            pwm->ctl = PWM_CTL_CLRF1;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            pwm->chn1Range = PWM_CHANNEL_RANGE;
            pwm->dmaConf = PWM_DMAC_ENAB | PWM_DMAC_PANIC(0x7) | PWM_DMAC_DREQ(0x7);
            pwm->ctl = PWM_CTL_USEF1 | PWM_CTL_RPTL1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
        }
        virtual ~PWMController() {
            pwm->ctl = 0x00000000;
        }
        inline volatile uint32_t &GetFifoIn() {
            return pwm->fifoIn;
        }
    private:
        volatile PWMRegisters *pwm;
};

class DMAController : public Device
{
    public:
        DMAController() = delete;
        DMAController(uint32_t address, unsigned dmaChannel) {
            dma = reinterpret_cast<DMARegisters *>(peripherals->GetVirtualAddress((dmaChannel < 15) ? DMA0_BASE_OFFSET + dmaChannel * 0x100 : DMA15_BASE_OFFSET));
            dma->ctlStatus = DMA_CS_RESET;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            dma->ctlStatus = DMA_CS_INT | DMA_CS_END;
            dma->cbAddress = address;
            dma->ctlStatus = DMA_CS_PANIC_PRIORITY(0xf) | DMA_CS_PRIORITY(0xf) | DMA_CS_ACTIVE;
        }
        virtual ~DMAController() {
            dma->ctlStatus = DMA_CS_RESET;
        }
        inline void SetControllBlockAddress(uint32_t address) {
            dma->cbAddress = address;
        }
        inline volatile uint32_t &GetControllBlockAddress() {
            return dma->cbAddress;
        }
    private:
        volatile DMARegisters *dma;
};

Transmitter::Transmitter()
    : output(nullptr), stop(true)
{
}

Transmitter::~Transmitter() {
    if (output != nullptr) {
        delete output;
    }
}

void Transmitter::Transmit(WaveReader &reader, float frequency, float bandwidth, unsigned dmaChannel, bool preserveCarrier)
{
    stop = false;

    WaveHeader header = reader.GetHeader();
    unsigned bufferSize = static_cast<unsigned>(static_cast<unsigned long long>(header.sampleRate) * BUFFER_TIME / 1000000);

    unsigned clockDivisor = static_cast<unsigned>(round(Peripherals::GetClockFrequency() * (0x01 << 12) / frequency));
    unsigned divisorRange = clockDivisor - static_cast<unsigned>(round(Peripherals::GetClockFrequency() * (0x01 << 12) / (frequency + 0.0005f * bandwidth)));

    if (output == nullptr) {
        output = new ClockOutput(clockDivisor);
    }

    auto finally = [&]() {
        if (!preserveCarrier) {
            delete output;
            output = nullptr;
        }
    };
    try {
        if (dmaChannel != 0xff) {
            TransmitViaDma(reader, *output, header.sampleRate, bufferSize, clockDivisor, divisorRange, dmaChannel);
        } else {
            TransmitViaCpu(reader, *output, header.sampleRate, bufferSize, clockDivisor, divisorRange);
        }
    } catch (...) {
        finally();
        throw;
    }
    finally();
}

void Transmitter::Stop()
{
    stop = true;
}

void Transmitter::TransmitViaCpu(WaveReader &reader, ClockOutput &output, unsigned sampleRate, unsigned bufferSize, unsigned clockDivisor, unsigned divisorRange)
{
    std::vector<Sample> samples = reader.GetSamples(bufferSize, stop);
    if (samples.empty()) {
        return;
    }

    unsigned sampleOffset = 0;
    bool eof = samples.size() < bufferSize, txStop = false;
    std::thread transmitterThread(Transmitter::TransmitterThread, this, &output, sampleRate, clockDivisor, divisorRange, &sampleOffset, &samples, &txStop);

    std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 2));

    auto finally = [&]() {
        {
            std::lock_guard<std::mutex> lock(access);
            txStop = true;
        }
        transmitterThread.join();
        samples.clear();
        stop = true;
    };
    try {
        while (!eof && !stop) {
            {
                std::lock_guard<std::mutex> lock(access);
                if (txStop) {
                    throw std::runtime_error("Transmitter thread has unexpectedly exited");
                }
                if (samples.empty()) {
                    if (!reader.SetSampleOffset(sampleOffset + bufferSize)) {
                        break;
                    }
                    samples = reader.GetSamples(bufferSize, stop);
                    if (samples.empty()) {
                        break;
                    }
                    eof = samples.size() < bufferSize;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 2));
        }
    } catch (...) {
        finally();
        throw;
    }
    finally();
}

void Transmitter::TransmitViaDma(WaveReader &reader, ClockOutput &output, unsigned sampleRate, unsigned bufferSize, unsigned clockDivisor, unsigned divisorRange, unsigned dmaChannel)
{
    if (dmaChannel > 15) {
        throw std::runtime_error("DMA channel number out of range (0 - 15)");
    }

    AllocatedMemory allocated(sizeof(uint32_t) * bufferSize + sizeof(DMAControllBlock) * (2 * bufferSize) + sizeof(uint32_t));

    std::vector<Sample> samples = reader.GetSamples(bufferSize, stop);
    if (samples.empty()) {
        return;
    }

    bool eof = false;
    if (samples.size() < bufferSize) {
        bufferSize = samples.size();
        eof = true;
    }

    PWMController pwm(sampleRate);
    Peripherals &peripherals = Peripherals::GetInstance();

    unsigned cbOffset = 0;

    volatile DMAControllBlock *dmaCb = reinterpret_cast<DMAControllBlock *>(allocated.GetBaseAddress());
    volatile uint32_t *clkDiv = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(dmaCb) + 2 * sizeof(DMAControllBlock) * bufferSize);
    volatile uint32_t *pwmFifoData = reinterpret_cast<uint32_t *>(reinterpret_cast<uintptr_t>(clkDiv) + sizeof(uint32_t) * bufferSize);
    for (unsigned i = 0; i < bufferSize; i++) {
        float value = samples[i].GetMonoValue();
        clkDiv[i] = CLK_PASSWORD | (0xffffff & (clockDivisor - static_cast<int32_t>(round(value * divisorRange))));
        dmaCb[cbOffset].transferInfo = DMA_TI_NO_WIDE_BURST | DMA_TI_WAIT_RESP;;
        dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(&clkDiv[i]);
        dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&output.GetDivisor());
        dmaCb[cbOffset].transferLen = sizeof(uint32_t);
        dmaCb[cbOffset].stride = 0;
        dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress(&dmaCb[cbOffset + 1]);
        cbOffset++;

        dmaCb[cbOffset].transferInfo = DMA_TI_NO_WIDE_BURST | DMA_TI_PERMAP(0x5) | DMA_TI_DEST_DREQ | DMA_TI_WAIT_RESP;
        dmaCb[cbOffset].srcAddress = allocated.GetPhysicalAddress(pwmFifoData);
        dmaCb[cbOffset].dstAddress = peripherals.GetPhysicalAddress(&pwm.GetFifoIn());
        dmaCb[cbOffset].transferLen = sizeof(uint32_t) * PWM_WRITES_PER_SAMPLE;
        dmaCb[cbOffset].stride = 0;
        dmaCb[cbOffset].nextCbAddress = allocated.GetPhysicalAddress((i < bufferSize - 1) ? &dmaCb[cbOffset + 1] : dmaCb);
        cbOffset++;
    }
    *pwmFifoData = 0x00000000;

    DMAController dma(allocated.GetPhysicalAddress(dmaCb), dmaChannel);

    std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 10));

    auto finally = [&]() {
        dmaCb[(cbOffset < 2 * bufferSize) ? cbOffset : 0].nextCbAddress = 0x00000000;
        while (dma.GetControllBlockAddress() != 0x00000000) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        samples.clear();
        stop = true;
    };
    try {
        while (!eof && !stop) {
            samples = reader.GetSamples(bufferSize, stop);
            if (!samples.size()) {
                break;
            }
            cbOffset = 0;
            eof = samples.size() < bufferSize;
            for (std::size_t i = 0; i < samples.size(); i++) {
                float value = samples[i].GetMonoValue();
                while (i == ((dma.GetControllBlockAddress() - allocated.GetPhysicalAddress(dmaCb)) / (2 * sizeof(DMAControllBlock)))) {
                    std::this_thread::sleep_for(std::chrono::microseconds(BUFFER_TIME / 10));
                }
                clkDiv[i] = CLK_PASSWORD | (0xffffff & (clockDivisor - static_cast<int>(round(value * divisorRange))));
                cbOffset += 2;
            }
        }
    } catch (...) {
        finally();
        throw;
    }
    finally();
}

void Transmitter::TransmitterThread(Transmitter *instance, ClockOutput *output, unsigned sampleRate, unsigned clockDivisor, unsigned divisorRange, unsigned *sampleOffset, std::vector<Sample> *samples, bool *stop)
{
    try {
        Peripherals &peripherals = Peripherals::GetInstance();

        volatile TimerRegisters *timer = reinterpret_cast<TimerRegisters *>(peripherals.GetVirtualAddress(TIMER_BASE_OFFSET));
        uint64_t current = *(reinterpret_cast<volatile uint64_t *>(&timer->low));
        uint64_t playbackStart = current;

        while (true) {
            std::vector<Sample> loadedSamples;
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(instance->access);
                    if (*stop) {
                        return;
                    }
                    loadedSamples = std::move(*samples);
                    current = *(reinterpret_cast<volatile uint64_t *>(&timer->low));
                    if (!loadedSamples.empty()) {
                        *sampleOffset = (current - playbackStart) * sampleRate / 1000000;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            };

            uint64_t start = current;
            unsigned offset = (current - start) * sampleRate / 1000000;

            while (true) {
                if (offset >= loadedSamples.size()) {
                    break;
                }
                unsigned prevOffset = offset;
                float value = loadedSamples[offset].GetMonoValue();
                instance->output->SetDivisor(clockDivisor - static_cast<int>(round(value * divisorRange)));
                while (offset == prevOffset) {
                    std::this_thread::sleep_for(std::chrono::microseconds(1)); // asm("nop");
                    current = *(reinterpret_cast<volatile uint64_t *>(&timer->low));;
                    offset = (current - start) * sampleRate / 1000000;
                }
            }
        }
    } catch (...) {
        std::lock_guard<std::mutex> lock(instance->access);
        *stop = true;
    }
}
