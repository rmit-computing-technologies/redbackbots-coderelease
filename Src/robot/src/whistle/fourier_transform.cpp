/**
 * @author Felix Weiglhofer
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */
#include "whistle/fourier_transform.h"

#include <cmath>
#include <cstring>


FourierTransform::FourierTransform(double * const input, 
        size_t inSize, float sampleRate)
    : _input(input)
    , _inSize(inSize)
    , _windowSize(inSize / 4)
    , _outSize(_windowSize / 2 + 1)
    , _fftInput( (fftw_complex *) fftw_malloc(_windowSize * sizeof(fftw_complex)))
    , _fftwOut( (fftw_complex *) fftw_malloc(_outSize * sizeof(fftw_complex)) )
    , _output(new double[_outSize])
    , _plan(fftw_plan_dft_r2c_1d(_windowSize, _input, _fftwOut, FFTW_ESTIMATE))
    , _freqSpacing((sampleRate / 2) / _outSize)
{
}

FourierTransform::~FourierTransform() {
    fftw_free(_fftInput);
    fftw_free(_fftwOut);
    delete[] _output;
}

float FourierTransform::length(const fftw_complex &c) {
    return sqrt( pow(c[0], 2) + pow(c[1], 2) );
}

void FourierTransform::execute() {
    
    for (int i = 0; i <= _inSize - _windowSize; i += _windowSize / 2) {
        // Copy windowed data to FFT input array
        std::memcpy(_fftInput, _input + i, sizeof(fftw_complex) * _windowSize);

        // Perform forward Fourier transform
        fftw_execute(_plan);
    }
    for (size_t i = 0; i < _outSize; i++) {
        
        _output[i] = log(length(_fftwOut[i]));
    }
}

// vim: set ts=4 sw=4 sts=4 expandtab:
