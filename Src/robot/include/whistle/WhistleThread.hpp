#pragma once

#include <string>

#include <portaudio.h>
#include <kissfft/kiss_fft.h>
#include <CompiledNN.h>
#include <asmjit/asmjit.h>

#include "blackboard/Adapter.hpp"
#include "blackboard/modules/WhistleBlackboard.hpp"
#include "types/RingBufferWithSum.h"
#include "utils/Timer.hpp"



/**
 * this class implements a thread that read audio samples from the microphones, then detect whether a whistle was blown.
 * it is based on nao devils 2023 whistle detector.
 */
class WhistleThread : Adapter {
   public:
      /* Constructor */
      WhistleThread(Blackboard *bb);
      /* Destructor */
      ~WhistleThread();

      /* One cycle of this thread */
      void tick();

   private:
      /**
       * Open microphone with the following preferred parameters:
       * - 4 mic channels 
       * - 22050hz sample rate
       * - Interleaved channels in the sample buffers
       * - little endian float32 [-1.0, 1.0] as the sample format
       */
      void openMic();

      /**
       * portaudio library call to fill in our audio buffer
       */
      void fillPortAudioBuffer();

      /**
       * kissfft library call to perform fft
       */
      void kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[]);

      /**
       * Sends the whistle detection event to the team
       */
      void updateTeamWhistle();

      /**
       * Get team whistle only if in the correct state
       * @return true if the team whistle is detected
       */
      bool getTeamWhistle();

      WhistleDetectionState whistleDetected;

      // CompiledNN
      NeuralNetwork::CompiledNN whistleNet;
      NeuralNetwork::Model whistleModel;
      asmjit::JitRuntime* jitRuntime;

      // PortAudio
      PaStream* stream;
      PaStreamParameters inputParameters;
      int sampleRate;
      int noDataCount; // this is used to check whether the stream has timed out.

      // kissfft
      std::vector<kiss_fft_cpx> in;
      std::vector<kiss_fft_cpx> out;
      
      // buffer sizes
      int nnInputSize;
      int timeDomainWindowSize;
      int freqDomainSize;
      
      // Broken Mic Detection
      unsigned int currentMic;
      bool allMicsBroken;

      // time domain buffers for whistle detection
      std::vector<float> portAudioBuffer;
      std::vector<float> windowBuffer; 
      int ringPos;
      int samplesLeft;

      // freq domain buffers (i.e. buffers for the FFT results)
      std::vector<float> amplitudes;
      std::vector<float> gradients;

      // buffers to hold information from previous whistle detector runs
      RingBufferWithSum<float> thresholdBuffer;
      RingBufferWithSum<float> confidenceBuffer;
      RingBufferWithSum<float,200> maxAmpHist;
      RingBufferWithSum<float,400> ampSumHist;
      RingBufferWithSum<int,10> whistleFreqBuffer;

      int detectedWhistleFrequency;

      unsigned int attackCount;
      unsigned int releaseCount;
      Timer attackTimeoutTimer;

      // configurable parameters (as seen in nao devils `WhistleDetector.cfg`)
      // TODO: figure out better variable names for these?
      int micBrokenThreshold;
      float noiseLimit;
      float confidenceThreshold;
      bool useAdaptiveThreshold;
      float adaptiveWindowSize; // size for the confidence ring buffers
      float nnWeight;
      float pmWeight;
      float noiseWeight;
      bool useWeightedPMConfidence;
      int minWhistleFreq;
      int maxWhistleFreq;
      bool freqCalibration;
      int release;
      int attack;
      uint32_t attackTimeout;

      // Whistle Restriction Settings
      float kickoffWhistleConfidence;
      float goalWhistleConfidence;
      bool actOnWhistleKickOff;
      bool actOnWhistleGoal;

};
