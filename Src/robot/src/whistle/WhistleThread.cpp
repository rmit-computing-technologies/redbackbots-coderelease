#include "whistle/WhistleThread.hpp"

// -- Blackboard Includes -- //
#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/WhistleBlackboard.hpp"
#include "blackboard/modules/ReceiverBlackboard.hpp"
#include "blackboard/modules/EventTransmitterBlackboard.hpp"
#include "blackboard/modules/EventReceiverBlackboard.hpp"

// -- Utils Includes -- //
#include "utils/Logger.hpp"
#include <boost/math/constants/constants.hpp>
#include <iomanip>
#include <ratio>

using boost::math::constants::two_pi;

// -- Whistle Comms -- //
const float STAY_IN_READY_SECS = 16.0; // GC delay is 15 seconds (2025 Rules)
const float MIN_STAY_IN_READY = 5.0; // If this amount of time has passed then can exit ready early

// The time bracket for events to be considered
const float MIN_COMMS_EVENT_WAIT = 0.00; // Check for events after this time
const float MAX_COMMS_EVENT_WAIT = 3.00; // Check for events before this time

WhistleThread::WhistleThread(Blackboard *bb)
    : Adapter(bb)
{
    // -- Import Configs from redbackbots.cfg -- //
    allMicsBroken = bb->config["whistle.allMicsBroken"].as<bool>();
    currentMic = bb->config["whistle.micChannel"].as<int>();
    noiseLimit = (bb->config)["whistle.noiseLimit"].as<float>();
    confidenceThreshold = (bb->config)["whistle.confidenceThreshold"].as<float>();
    useAdaptiveThreshold = (bb->config)["whistle.useAdaptiveThreshold"].as<bool>();
    adaptiveWindowSize = (bb->config)["whistle.adaptiveWindowSize"].as<float>(); // size for the confidence ring buffers
    nnWeight = (bb->config)["whistle.nnWeight"].as<float>();
    pmWeight = (bb->config)["whistle.pmWeight"].as<float>();
    noiseWeight = (bb->config)["whistle.noiseWeight"].as<float>();
    useWeightedPMConfidence = (bb->config)["whistle.useWeightedPMConfidence"].as<bool>();
    minWhistleFreq = (bb->config)["whistle.minWhistleFreq"].as<int>();
    maxWhistleFreq = (bb->config)["whistle.maxWhistleFreq"].as<int>();
    freqCalibration = (bb->config)["whistle.freqCalibration"].as<bool>();
    release = (bb->config)["whistle.release"].as<int>();
    attack = (bb->config)["whistle.attack"].as<int>();
    attackTimeout = (bb->config)["whistle.attackTimeout"].as<int>();

    // -- Whistle Comms Configs -- //
    kickoffWhistleConfidence = (bb->config)["whistle.confidence.kickoff"].as<float>();
    goalWhistleConfidence = (bb->config)["whistle.confidence.goal"].as<float>();
    actOnWhistleKickOff = (bb->config)["whistle.whistle.kickoff"].as<bool>();
    actOnWhistleGoal = (bb->config)["whistle.whistle.goal"].as<bool>();

    if (!actOnWhistleKickOff) {
        llog(WARNING) << "!!!!! Kickoff whistle detection is disabled !!!!!" << std::endl;
    }
    if (!actOnWhistleGoal) {
        llog(WARNING) << "!!!!! Goal scored whistle detection is disabled !!!!!" << std::endl;
    }

    if (allMicsBroken)
    {
        // vocalize that whistle detector is not working.
        // TODO: make this functionality into a macro that other cpp files can use. then remove `speech.cpp`
        system("echo '{\"message\": \"ALL mics set to broken. I am not hearing whistles.\",\"level\": \"INFO\"}' | nc -q 0 localhost 65432");
    }
    this->openMic();

    // NOTE: we might not want to start another jitruntime here in whistle thread after we merge with vision branch...
    jitRuntime = new asmjit::JitRuntime();

    whistleModel.load("/home/nao/data/whistle/NaoDevilsWhistleNet.h5");
    whistleNet.compile(whistleModel);
    if (!whistleNet.valid())
    {
        llog(ERROR) << "failed to compile whistleModel" << std::endl;
    }

    noDataCount = 0;

    // calculate buffer sizes and allocate buffers
    nnInputSize = whistleNet.input(0).size();
    timeDomainWindowSize = (nnInputSize * 2) - 2;
    freqDomainSize = nnInputSize;

    portAudioBuffer = std::vector<float>();
    in = std::vector<kiss_fft_cpx>(timeDomainWindowSize);
    out = std::vector<kiss_fft_cpx>(timeDomainWindowSize);
    windowBuffer = std::vector<float>(timeDomainWindowSize);
    ringPos = 0;
    samplesLeft = timeDomainWindowSize;

    amplitudes = std::vector<float>(freqDomainSize);
    gradients = std::vector<float>(freqDomainSize);

    releaseCount = static_cast<unsigned int>(release);
    attackCount = 1;
    attackTimeoutTimer = Timer();

    thresholdBuffer.reserve(adaptiveWindowSize);
    confidenceBuffer.reserve((size_t)std::ceil(adaptiveWindowSize / 2));
}

WhistleThread::~WhistleThread()
{
    // close the portaudio stream
    if (stream)
    {
        if (Pa_IsStreamActive(stream))
            Pa_StopStream(stream);

        Pa_CloseStream(stream);
        stream = nullptr;
    }

    Pa_Terminate();

    delete jitRuntime;
}

void WhistleThread::openMic()
{
    // preffered mic configs
    double preferredSampleRate = 22050;
    int preferredChannels = 4;

    PaError paerr = Pa_Initialize();
    if (paerr != paNoError)
    {
        llog(ERROR) << "Failed to initialize PortAudio: " << Pa_GetErrorText(paerr) << "(" << paerr << ")" << std::endl;
        return;
    }

    inputParameters.device = Pa_GetDefaultInputDevice();
    if (inputParameters.device == paNoDevice)
    {
        llog(ERROR) << "PortAudio: No input device!" << std::endl;
        return;
    }

    // use the device with 4 mic channels if possible
    for (int dev = 0; dev < Pa_GetDeviceCount(); dev++)
    {
        if (Pa_GetDeviceInfo(dev)->name == std::string("multi"))
        {
            inputParameters.device = dev;
        }
    }

    const PaDeviceInfo* info = Pa_GetDeviceInfo(inputParameters.device);

    inputParameters.channelCount = std::min(preferredChannels, info->maxInputChannels);
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = 0.1f;
    inputParameters.hostApiSpecificStreamInfo = nullptr;

    paerr = Pa_OpenStream(
        &stream,
        &inputParameters,
        nullptr, // this arg is only used if we're trying to interact with speakers
        preferredSampleRate,
        paFramesPerBufferUnspecified,
        0,       // this arg is only used if we're opening the stream with specific flags
        nullptr, // this arg is only used if we're using callback functions
        nullptr  // this arg is only used if we're using callback functions
    );

    if (paerr != paNoError)
    {
        llog(ERROR) << "PortAudio: Pa_OpenStream failed: " << Pa_GetErrorText(paerr) << "(" << paerr << ")" << std::endl;
        return;
    }

    this->sampleRate = preferredSampleRate;
}

void WhistleThread::fillPortAudioBuffer()
{
    portAudioBuffer.clear();
    PaError paerr;

    if (stream == nullptr)
        return;

    // start stream if stopped
    if (Pa_IsStreamStopped(stream))
    {
        paerr = Pa_StartStream(stream);
        if (paerr != paNoError)
        {
            llog(ERROR) << "PortAudio: Pa_StartStream failed: " << Pa_GetErrorText(paerr) << "(" << paerr << ")" << std::endl;
            Pa_CloseStream(stream);
            stream = nullptr;
            return;
        }
    }

    signed long available = Pa_GetStreamReadAvailable(stream);
    if (available < 0)
    {
        llog(ERROR) << "PortAudio: Pa_GetStreamReadAvailable failed: " << Pa_GetErrorText(static_cast<PaError>(available)) << "(" << static_cast<PaError>(available) << ")" << std::endl;
        return;
    }

    // stop stream if no data is available over a longer period of time, start again in the next tick
    if (available == 0)
    {
        if (++(this->noDataCount) > 30)
        {
            llog(WARNING) << "PortAudio: Data timeout, restarting stream" << std::endl;
            Pa_StopStream(stream);
            this->noDataCount = 0;
        }
        return;
    }
    this->noDataCount = 0;

    portAudioBuffer.resize(available * this->inputParameters.channelCount);
    paerr = Pa_ReadStream(stream, portAudioBuffer.data(), available);
    if (paerr != paNoError)
    {
        llog(ERROR) << "PortAudio: Pa_ReadStream failed: " << Pa_GetErrorText(paerr) << "(" << paerr << ")" << std::endl;
        portAudioBuffer.clear();

        // stop stream immediately after timeout, start again in the next tick.
        if (paerr == paTimedOut)
            Pa_StopStream(stream);
        return;
    }
}

void WhistleThread::tick()
{
    // fully rely on team whistle if all mics are broken.
    if (this->allMicsBroken)
    {
        this->whistleDetected = WhistleDetectionState::dontKnow;
        // Only send team whistle if gc is running
        if (readFrom(gameController, active))
        {
            this->updateTeamWhistle();

            this->whistleDetected = this->getTeamWhistle() ? WhistleDetectionState::isDetected : WhistleDetectionState::notDetected;
        }

        // write to blackboard
        writeTo(whistle, whistleDetectionState, this->whistleDetected);
        return;
    }

    this->fillPortAudioBuffer();

    unsigned int audioDataPos = currentMic;
    while (audioDataPos < portAudioBuffer.size())
    {
        while (audioDataPos < portAudioBuffer.size() && samplesLeft > 0)
        {
            windowBuffer[ringPos] = portAudioBuffer[audioDataPos];

            audioDataPos += inputParameters.channelCount;
            samplesLeft--;
            ringPos = (ringPos + 1) % timeDomainWindowSize;
        }

        if (samplesLeft == 0)
        {
            samplesLeft = timeDomainWindowSize;

            // prepare windowBuffer into fft in buffer
            for (unsigned int i = 0; i < static_cast<unsigned int>(timeDomainWindowSize); i++)
            {
                in[i].r = windowBuffer[(i + ringPos) % timeDomainWindowSize];
                in[i].i = 0.f;

                // apply Hamming window
                in[i].r *= (0.54f - 0.46f * std::cos(two_pi<float>() * i / timeDomainWindowSize));
            }

            // do FFT analysis
            this->kissFFT(in.data(), out.data());

            // analyze the FFT output
            float currentMaxAmp = 0;
            float currentMeanAmp = 0;
            float prevAmp = 0;
            float ampSum = 0;
            float loudFreqsCount = 0;
            for (int i = 0; i < freqDomainSize; i++)
            {
                float amp = std::sqrt((out[i].r * out[i].r) + (out[i].i * out[i].i));

                whistleNet.input(0)[i] = 20 * std::log10(amp);
                amplitudes[i] = amp;
                gradients[i] = amp - prevAmp;
                prevAmp = amp;

                if (amp > currentMaxAmp)
                    currentMaxAmp = amp;

                if (amp > noiseLimit)
                    loudFreqsCount++;

                ampSum = ampSum + amp;
            }
            float loudFreqsRatio = loudFreqsCount / freqDomainSize; // ratio between loud & quiet/noisy frequencies in the FFT result
            maxAmpHist.push_front(currentMaxAmp);
            currentMeanAmp = ampSum / freqDomainSize;
            ampSumHist.push_front(ampSum);

            // do WHistleDetection PM
            int minPos = minWhistleFreq * timeDomainWindowSize / sampleRate;
            int maxPos = maxWhistleFreq * timeDomainWindowSize / sampleRate;

            // find whistle peak between min. freq. position and  max. freq. position
            int peakPos = minPos;
            for (int i = minPos; i <= maxPos; i++)
            {
                if (amplitudes[i] > amplitudes[peakPos])
                    peakPos = i;
            }

            float ampWeight = 1.f;
            if (useWeightedPMConfidence)
                // weigh down PM if the loudest frequency isnt in the whistle frequency range
                ampWeight = (amplitudes[peakPos] / currentMaxAmp);

            // get min/max gradients around peakPos
            float maxGrad = gradients[peakPos];
            float minGrad = gradients[std::min(peakPos + 1, freqDomainSize - 1)];
            maxGrad = std::abs(maxGrad / currentMaxAmp);
            minGrad = std::abs(minGrad / currentMaxAmp);

            float pmConfidence;
            if (amplitudes[peakPos] >= maxAmpHist.average())
            {
                minPos = static_cast<int>(minPos * 2);
                maxPos = static_cast<int>(maxPos * 2);

                // find first overtone peak between peak freq. position * overtoneMultiplierMin  and peak freq. position * overtoneMultiplierMax
                int overtonePeakPos = minPos;
                for (int i = minPos; i <= maxPos; i++)
                {
                    if (amplitudes[i] > amplitudes[overtonePeakPos])
                        overtonePeakPos = i;
                }

                //detect whistle
                if (amplitudes[overtonePeakPos] >= currentMeanAmp)
                {
                    pmConfidence = std::max(minGrad, maxGrad) * ampWeight;
                }
                else
                {
                    pmConfidence = std::min(minGrad, maxGrad) * ampWeight;
                }
            }
            else
            {
                pmConfidence = std::min(minGrad, maxGrad) * ampWeight;
            }

            // do WhistleDetection NN
            whistleNet.apply();
            float nnConfidence = whistleNet.output(0)[0];

            // Merge NN, PM and limit information
            float confidence = ((nnWeight * nnConfidence + pmWeight * pmConfidence) / (nnWeight + pmWeight)) * (1 - (noiseWeight * loudFreqsRatio));
            confidenceBuffer.push_front(confidence);

            if(nnConfidence > 0.8)
                llog(DEBUG)
                    << "NaoDevils Whistle Detector Debug information:" << std::endl
                    << "\t- nnConfidence: " << std::fixed << std::setprecision(6) << nnConfidence << std::endl
                    << "\t- pmConfidence: " << std::fixed << std::setprecision(6) << pmConfidence << std::endl
                    << "\t- mergedConfidence: " << std::fixed << std::setprecision(6) << confidence << std::endl
                    << std::endl;

            if (useAdaptiveThreshold)
                thresholdBuffer.push_front(confidenceThreshold + ((1 - confidenceThreshold) * (noiseWeight * loudFreqsRatio)));
            else
                thresholdBuffer.push_front(confidenceThreshold);

            // notice that we are using `confidenceBuffer.back()` here
            this->whistleDetected = confidenceBuffer.back() > thresholdBuffer.average() ? WhistleDetectionState::isDetected : WhistleDetectionState::notDetected;

            // freq calibration
            if (this->whistleDetected == WhistleDetectionState::isDetected)
            {
                detectedWhistleFrequency = peakPos * sampleRate / timeDomainWindowSize;
                if (freqCalibration && detectedWhistleFrequency > minWhistleFreq && detectedWhistleFrequency < maxWhistleFreq)
                {
                    if (confidenceBuffer.back() > thresholdBuffer.average() * 1.5f)
                    {
                        llog(INFO) << "Update whistle frequency." << std::endl;
                        whistleFreqBuffer.push_front(detectedWhistleFrequency);
                        int minDiff = std::abs(whistleFreqBuffer.average() - minWhistleFreq);
                        int maxDiff = std::abs(whistleFreqBuffer.average() - maxWhistleFreq);

                        // TODO: recheck the naodevils codebase to see the intended use of freqCalibration.
                        //       do we want to save the new values back to the config file?
                        minWhistleFreq = std::min(whistleFreqBuffer.average() - minDiff / 2, whistleFreqBuffer.average() - 250);
                        maxWhistleFreq = std::max(whistleFreqBuffer.average() + maxDiff / 2, whistleFreqBuffer.average() + 250);
                    }
                }
            }
        }
    }


    // only write to the blackboard that we've detected a whistle, if we've detected whistles for `attack` ticks in a row.
    if (this->whistleDetected == WhistleDetectionState::isDetected)
    {
        attackCount = attackTimeoutTimer.elapsed_ms() < attackTimeout ? attackCount+1 : 1;
        attackTimeoutTimer.restart();

        // revert back to notDetected if we dont have enough attack
        if (attackCount < static_cast<unsigned int>(attack))
        {
            this->whistleDetected = WhistleDetectionState::notDetected;
        }
    }

    // sustain positive whistle detection for `release` ticks
    if (this->whistleDetected == WhistleDetectionState::isDetected)
    {
        releaseCount = 0;
        llog(INFO)
            << "############################################################" << std::endl
            << "## Whistle Detected in the new naodevils whistle detector ##" << std::endl
            << "############################################################" << std::endl;

        std::string out = "echo '{\"message\": \"Ego Whistle Heard ";
        int myNum = readFrom(gameController, player_number);
        out += std::to_string(myNum);
        out += "\",\"level\": \"INFO\"}' | nc -q 0 localhost 65432";
        system(out.c_str());
    }
    else if (releaseCount < static_cast<unsigned int>(release))
    {
        this->whistleDetected = WhistleDetectionState::isDetected;
        releaseCount++;
    }

    // Only send team whistle if gc is running
    if (readFrom(gameController, active))
    {
        this->updateTeamWhistle();

        this->whistleDetected = this->getTeamWhistle() ? WhistleDetectionState::isDetected : WhistleDetectionState::notDetected;

        if (this->whistleDetected == WhistleDetectionState::isDetected)
        {
            std::string out = "echo '{\"message\": \"Team Whistle Heard ";
            int myNum = readFrom(gameController, player_number);
            out += std::to_string(myNum);
            out += "\",\"level\": \"INFO\"}' | nc -q 0 localhost 65432";
            system(out.c_str());
        }
    }

    // write to blackboard
    writeTo(whistle, whistleDetectionState, this->whistleDetected);
}

void WhistleThread::updateTeamWhistle()
{
    if (this->whistleDetected == isDetected)
    {
        // Send the event immediately
        EventTransmitter.raiseEvent("WHISTLE_HEARD", (int)WhistleDetectionState::isDetected, 0.0f);
    }
}

bool WhistleThread::getTeamWhistle()
{
    // -- Get Communication Events -- //
    EventReceiverBlackboard &eventReceiver = EventReceiver;

    uint8_t gameState = readFrom(gameController, gameState);
    float minRequiredConfidence;

    if (gameState == STATE_SET && actOnWhistleKickOff)
    {
        minRequiredConfidence = kickoffWhistleConfidence;
    }
    else if (gameState == STATE_PLAYING && actOnWhistleGoal)
    {
        minRequiredConfidence = goalWhistleConfidence;
    }
    else {
        return false;
    }


    // EXTENSION: Give a greater weighting to robots closer to the ref position
        // Start points (from older code)
        //     AbsCoord robot_pos = readFrom(stateEstimation, robotPos);
        //     bool nearCenterCircle = abs(robot_pos.x()) < 1500 && abs(robot_pos.y()) < 3000;
    int totalHeardWhistle = 0;
    int numActive = 0;

    for (int playerNum = 1; playerNum <= ROBOTS_PER_TEAM; ++playerNum) {

        float timeSinceWhistleEvent = eventReceiver.getEventTimeSinceReceived(playerNum, "WHISTLE_HEARD");

        int default_value = (int)WhistleDetectionState::dontKnow; // Outside delcaration is needed to prevent type error
        int received = eventReceiver.getEventData(playerNum, "WHISTLE_HEARD")->getUnpackedValue(default_value);

        // check if received value is in the range of valid enum states, else fallback to `dontKnow`
        WhistleDetectionState state = WhistleDetectionState::dontKnow;
        if (received >= 0 && received <= 2)
        {
            state = static_cast<WhistleDetectionState>(received);
        }

        if ( !(readFrom(receiver, incapacitated)[playerNum] || state == WhistleDetectionState::dontKnow) ) {
            numActive ++;
        }
        if ( (MIN_COMMS_EVENT_WAIT <= timeSinceWhistleEvent && timeSinceWhistleEvent < MAX_COMMS_EVENT_WAIT)
            && state == WhistleDetectionState::isDetected) {
            totalHeardWhistle ++;
        }
    }
    bool teamWhistleDetected = minRequiredConfidence <= (static_cast<float>(totalHeardWhistle) / (numActive != 0 ? numActive : 1));

    llog(INFO) << "Team Whistle Detected: " << teamWhistleDetected << std::endl;
    llog(DEBUG) << "Num Active: " << numActive << std::endl;
    llog(DEBUG) << "Total Heard Whistle: " << totalHeardWhistle << std::endl;

    return teamWhistleDetected;
}

void WhistleThread::kissFFT(kiss_fft_cpx in[], kiss_fft_cpx out[])
{
    kiss_fft_cfg cfg;

    if ((cfg = kiss_fft_alloc(this->timeDomainWindowSize, 0 /*is_inverse_fft*/, NULL, NULL)) != NULL)
    {
        kiss_fft(cfg, in, out);
        free(cfg);
    }
    else
    {
        llog(WARNING) << "Not enough memory for KissFFT?" << std::endl;
    }
}
