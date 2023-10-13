/*!
 * \brief An interface to ALSA for recording sound data.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 * \author Felix Weiglhofer
 */
#pragma once

#include "audiostream.h"

#include <cstdint>
#include <string>
#include <alsa/asoundlib.h>

class WhistleSettings;

class AlsaRecorder : public AudioStream {
public:

    struct Settings {
        size_t bufferSize;
        unsigned int channelNum;
        float sampleRate;
        std::string soundDevice;
        std::string soundDeviceVol;
        std::string soundSubDevice;
        int volume;
    };

    /**
     * Default settings for NaoV5.
     */
    static const Settings V5_SETTINGS;

    AlsaRecorder(const Settings &);
    ~AlsaRecorder();

    // AudioStream interface
    void start() override;
    void stop() override;
    bool fetch() override;
    int16_t *getBuffer() override;
    size_t getBufferSize() override;
    short getChannelNum() override;
    double getSampleRate() override;

    bool isRunning() const;

private:
    const Settings settings;

    int16_t *audioBuffer;
    snd_pcm_t *captureHandle;

    void setVolume(const char *subdevice);
    void destroyAlsa();

    int xrunRecovery(snd_pcm_t *handle, int err);

    void createAudioBuffer();
};

// vim: set ts=4 sw=4 sts=4 expandtab:
