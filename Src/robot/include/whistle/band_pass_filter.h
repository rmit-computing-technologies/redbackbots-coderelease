/**
 * @author Felix Weiglhofer
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */
#pragma once

#include <cstddef>
#include "filt.h"

class BandPassFilter {
    
public:

    /**
     * @param smoothWidth: Number of adjacent points to build the average of.
     * Must be uneven.
     * @param input: Input data.
     * @param inSize: Size of input.
     * @param freqSpacing: Spacing between points in input.
     */
    explicit BandPassFilter(unsigned int smoothWidth,  
            const double * const input, size_t inSize, float freqSpacing, float sampleRate); 

    ~BandPassFilter();

    void execute();

    const double *output() { return _output; }
    size_t outSize() { return _outSize; }

    float freqSpacing() { return _freqSpacing; }
    float freqOffset()  { return _freqOffset; }

private:

    const double * const _input;
    const size_t _inSize;

    double *_output;
    size_t _outSize;

    unsigned int _smoothWidth;

	Filter *my_filter;

    float _freqSpacing;
    float _freqOffset;

};

// vim: set ts=4 sw=4 sts=4 expandtab:
