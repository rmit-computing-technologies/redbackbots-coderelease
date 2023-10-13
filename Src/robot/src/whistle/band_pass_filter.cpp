/**
 * @author Felix Weiglhofer
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */
#include "whistle/band_pass_filter.h"

#include "whistle/filt.h"
#include <iostream>

#include <cassert>
#include <iterator>
#include <algorithm>
#include <unistd.h>

BandPassFilter::BandPassFilter(unsigned int smoothWidth, 
        const double * const input, 
        size_t inSize, float freqSpacing, float sampleRate) 
    : _input(input)
    , _inSize(inSize)
    , _output(nullptr)
    , _outSize(0)
    , _smoothWidth(smoothWidth)
    , _freqSpacing(freqSpacing)
{
    assert(_input != nullptr);
    assert(_smoothWidth % 2 == 1);
    assert(_inSize >= _smoothWidth);

    unsigned int cutOff = (_smoothWidth * 2) - 1;

    _outSize = _inSize ; //- cutOff;
    _output  = new double[_outSize];

    _freqOffset = (cutOff / 2) * _freqSpacing;
    sampleRate = sampleRate/1000;
    float Fu = 12.0;                    // Upper cutoff frequency
    float Fl = 8.0;                     // Lower cutoff frequency
    float Fc = (Fu+Fl) / 2;             // Center frequency
    float Bw = Fc*0.10;                 // Bandwidth
    int num_taps = 4 * sampleRate / Bw; // Number of taps
    my_filter = new Filter(BPF,num_taps, sampleRate, Fl, Fu);
}

BandPassFilter::~BandPassFilter() {
    delete[] _output;
}

void BandPassFilter::execute() {
    FILE* fd_out;
    fd_out = fopen("referee.raw", "a");
    short samp_dat;
    double out_val;
    for (size_t inputSam = 0; inputSam < _outSize; inputSam++) {
        out_val = my_filter->do_sample( (double) _input[inputSam] );
        _output[inputSam] = out_val;
        samp_dat = (short)out_val;
        fwrite(&samp_dat, sizeof(short), 1, fd_out);
    }
    fclose(fd_out);
}

// vim: set ts=4 sw=4 sts=4 expandtab:
