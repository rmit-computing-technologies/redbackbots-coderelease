/**
 * @author Felix Weiglhofer
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */
#pragma once

#include <cstddef>

#include "tiny_dnn/tiny_dnn.h"
#include <CompiledNN.h>

#include "perception/vision/detector/DNNHelper.hpp"

#define DNN_WEIGHTS_DIR "/home/nao/data/dnn_model/whistle_weights/"

using namespace NeuralNetwork;

/**
 * Classifies audio data as a whistle or not.
 */
class WhistleClassifier {
    
public:

    /**
     * @param input: Audio data in frequency domain.
     * @param inSize: Size of input.
     * @param whistleThreshold: Classify input as a whistle if the peak in
     * input is higher than whistleThreshold (Unit: Hz)
     * @param minWhistleLength: Only accept a signal as a whistle if its at
     * least minWhistleLength samples long.
     * @param bufferLength: Number of audio samples that is processed with
     * every processing step.
     * @param freqSpacing: Spacing between entries in input.
     * @param freqOffset: Frequency offset of the first entry of input.
     */
    explicit WhistleClassifier(
            const double * const input, 
            size_t inSize, 
            float whistleThreshold,
            unsigned int minWhistleLength, 
            float bufferLength,
            float freqSpacing,
            float freqOffset);

    void execute();

    bool whistleDetected() { return _whistleDetected; }

    bool hasCandidate() { return _matchLength > 0; }

    float currPeak() { return _currPeak; }

    void reset(); 

private:
    const double * const _input;
    const size_t _inSize;

    const float _bufferLength;
    const float _freqSpacing;
    const float _freqOffset;

    size_t _matchLength;
    const size_t _minMatchLength;
    float _currPeak;

    const float _whistleThreshold;

    bool _whistleDetected;

    float findPeak();

    tiny_dnn::network<tiny_dnn::sequential> createModel();

    // Specify the input shape
    const int _inputWidth;
    const int _inputHeight;
    const int _inputChannels; // Grayscale

    // Specify the number of output classes
    const int _numClasses; // Whistle or Non-whistle
    Model _model;
    CompiledNN _nn;
    tiny_dnn::network<tiny_dnn::sequential> _net;
};

// vim: set ts=4 sw=4 sts=4 expandtab:
