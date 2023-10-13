#pragma once

#include <cstdint>
#include <cstddef>

class AudioStream {
public:
    virtual ~AudioStream() {};
    virtual int16_t *getBuffer() = 0;
    virtual size_t getBufferSize() = 0;
    virtual short getChannelNum() = 0;
    virtual double getSampleRate() = 0;
    virtual bool fetch() = 0;
    virtual void stop() = 0;
    virtual void start() = 0;
};
