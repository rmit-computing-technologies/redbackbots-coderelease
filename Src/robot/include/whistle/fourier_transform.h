/**
 * @author Felix Weiglhofer
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */
#pragma once

#include <fftw3.h>

/**
 * Performs a Short Term fourier transform on the input data.
 */
class FourierTransform {
    
public:
    /**
     * @param input: Input signal.
     * @param inSize: Size of input.
     * @param sampleRate: Samplerate of input signal.
     */
    explicit FourierTransform(double * const input, size_t inSize, float sampleRate);
    ~FourierTransform();

    void execute();

    const double *output() { return _output; }
    size_t outSize() { return _outSize; }

    size_t getWindowSize() { return _windowSize; }

    float freqSpacing() { return _freqSpacing; }

private:
    static inline float length(const fftw_complex &);

    double * const _input;
    const size_t _inSize;
    const size_t _windowSize;

    const size_t _outSize;
    fftw_complex * const _fftInput;
    fftw_complex * const _fftwOut; 
    double * const _output; 

    const fftw_plan _plan;

    const float _freqSpacing;

};

// vim: set ts=4 sw=4 sts=4 expandtab:
