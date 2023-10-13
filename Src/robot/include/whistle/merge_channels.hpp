/**
 * @author Felix Weiglhofer
 */
#pragma once

#include <cassert>
#include <cstdint>
#include <cstddef>


/**
 * Merge audio data from multiple channels.
 */
template<void * (*customMalloc)(size_t), void (*customFree)(void *)>
class MergeChannels {
    
public:
    explicit MergeChannels(const int16_t * const input, size_t inSize, unsigned int channelNum)
        : _input(input)
        , _inSize(inSize)
        , _outSize(inSize / channelNum)
        , _output( (double *) customMalloc(_outSize * sizeof(double)) )
        , _channelNum(channelNum)
    {
        assert(_inSize % _channelNum == 0);
    }

    ~MergeChannels() {
        customFree(_output);
    }

    void execute() {
        for (size_t i = 0; i < _outSize; i++) {
            _output[i] = 0;
            for (unsigned int j = 0; j < _channelNum; j++) {
                _output[i] += _input[i * _channelNum + j];
            }
            _output[i] /= _channelNum;
        }
    }

    double *output() { return _output; }
    size_t outSize() { return _outSize; }

private:

    const int16_t * const _input;
    const size_t _inSize;

    const size_t _outSize;
    double * const _output;

    const unsigned int _channelNum;

};

// vim: set ts=4 sw=4 sts=4 expandtab:
