/*!
 * \brief An interface to ALSA for recording sound data.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 * \author Felix Weiglhofer
 */


#include "whistle/alsarecorder.h"

#include <iostream>
#include <cassert>

#include <cmath>

const AlsaRecorder::Settings AlsaRecorder::V5_SETTINGS = {
    2048,
    1,
    44100,
    "default",
    "default",
    "Capture",
    100
};

AlsaRecorder::AlsaRecorder(const Settings &s)
    : settings(s)
    , audioBuffer(nullptr)
    , captureHandle(nullptr)
{
    createAudioBuffer();
}

AlsaRecorder::~AlsaRecorder() {
    destroyAlsa();
    delete[] audioBuffer;
}

void AlsaRecorder::createAudioBuffer() {
    assert(settings.channelNum > 0);
    assert(settings.sampleRate > 0);
    audioBuffer = new int16_t[getChannelNum() * getBufferSize()];
}

void AlsaRecorder::stop() {
    destroyAlsa();
}

bool AlsaRecorder::fetch() {
    int err;
    assert(captureHandle != nullptr);
    assert(audioBuffer != nullptr);
    while (true) {
        if ( (err = snd_pcm_readi(captureHandle, audioBuffer, getBufferSize())) == static_cast<int>(getBufferSize())) {
            return true;
        }
        std::cerr << "read from audio interface failed " << snd_strerror(err) << std::endl;
        /* try to recover */
        if ( (err = xrunRecovery(captureHandle, err)) !=  0 ) {
            std::cerr <<  "couldn't recover " << snd_strerror(err) << std::endl;
            return false;
        }
    }
}

int16_t *AlsaRecorder::getBuffer() {
    return audioBuffer;
}

size_t AlsaRecorder::getBufferSize() {
    return settings.bufferSize; 
}

short AlsaRecorder::getChannelNum() {
    return settings.channelNum;
}

double AlsaRecorder::getSampleRate() {
    return settings.sampleRate;
}

/*******************************************************************/
/**
 * Must be called before reading data.
 */
void AlsaRecorder::start()
{
    assert(captureHandle == nullptr);

    int err;
    snd_pcm_hw_params_t *hwParams;

    if((err = snd_pcm_open(&captureHandle, settings.soundDevice.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
        std::cerr <<  "cannot open audio device " << settings.soundDevice << "(" << snd_strerror(err) << ")" << std::endl;
        return;
    }

    if((err = snd_pcm_hw_params_malloc(&hwParams)) < 0) {
        std::cerr <<  "cannot allocate hardware parameter structure " << snd_strerror(err) << std::endl;
        return;
    }

    if((err = snd_pcm_hw_params_any(captureHandle, hwParams)) < 0) {
        std::cerr <<  "cannot initialize hardware parameter structure " << snd_strerror(err) << std::endl;
        return;
    }

    if((err = snd_pcm_hw_params_set_access(captureHandle, hwParams, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        std::cerr <<  "cannot set access type " << snd_strerror(err) << std::endl;
        return;
    }

    if((err = snd_pcm_hw_params_set_format(captureHandle, hwParams, SND_PCM_FORMAT_S16_LE)) < 0) {
        std::cerr <<  "cannot set sample format " << snd_strerror(err) << std::endl;
        return;
    }

    unsigned int oSR;
    oSR = getSampleRate();
    if((err = snd_pcm_hw_params_set_rate_near(captureHandle, hwParams, &oSR, 0)) < 0) {
        std::cerr <<  "cannot set sample rate " << snd_strerror(err) << std::endl;
        return;
    }

    // if(oSR != getSampleRate()) {
    //     std::cerr <<  "cannot set sample rate, correct should be: " << getSampleRate() << ", is " << oSR << std::endl;
    //     return;
    // }

    std::cerr <<  "ALSA-RX opened with a samplerate of " << oSR << " (requested: " << getSampleRate() << ")." << std::endl;

    if((err = snd_pcm_hw_params_set_channels(captureHandle, hwParams, getChannelNum())) < 0) {
        std::cerr <<  "cannot set channel count " << snd_strerror(err) << std::endl;
        return;
    }

    if((err = snd_pcm_hw_params(captureHandle, hwParams)) < 0) {
        std::cerr <<  "cannot set parameters " << snd_strerror(err) << std::endl;
        return;
    }

    snd_pcm_hw_params_free(hwParams);

    if((err = snd_pcm_prepare(captureHandle)) < 0) {
        std::cerr <<  "cannot prepare audio interface for use " << snd_strerror(err) << std::endl;
        return;
    }

    setVolume(settings.soundSubDevice.c_str());
}

void AlsaRecorder::setVolume(const char *subdevice)
{
    int err;
    long vmin;
    long vmax;
    snd_mixer_t *mixer;
    snd_mixer_selem_id_t *selemid;

    int mode = 0;
    if((err = snd_mixer_open(&mixer, mode)) < 0) {
        std::cerr <<  "unable to open mixer " << snd_strerror(err) << std::endl;
        return;
    }
    if((err = snd_mixer_attach(mixer, settings.soundDeviceVol.c_str())) < 0) {
        std::cerr <<  "unable to attach card to mixer " << snd_strerror(err) << std::endl;
        return;
    }
    if((err = snd_mixer_selem_register(mixer, NULL, NULL)) < 0) {
        std::cerr <<  "unable to register mixer " << snd_strerror(err) << std::endl;
        return;
    }
    if((err = snd_mixer_load(mixer)) < 0) {
        std::cerr <<  "unable to load mixer " << snd_strerror(err) << std::endl;
        return;
    }
    snd_mixer_selem_id_malloc(&selemid);
    if(selemid == NULL) {
        std::cerr <<  "unable to allocate selemid." << std::endl;
        return;
    }
    snd_mixer_selem_id_set_index(selemid, 0);
    snd_mixer_selem_id_set_name(selemid, subdevice);
    snd_mixer_elem_t* elem = snd_mixer_find_selem(mixer, selemid);
    if(elem == NULL) {
        std::cerr <<  "unable to find selem." << std::endl;
        return;
    }
    if((err = snd_mixer_selem_get_capture_volume_range(elem, &vmin, &vmax)) < 0) {
        std::cerr <<  "unable to get capture volume range " << snd_strerror(err) << std::endl;
        return;
    }
    if((err = snd_mixer_selem_set_capture_volume_all(elem, (vmax * settings.volume) / 100)) < 0) {
        std::cerr <<  "unable to set capture volume " << snd_strerror(err) << std::endl;
        return;
    }
    snd_mixer_selem_id_free(selemid);
    if((err = snd_mixer_close(mixer)) < 0) {
        std::cerr <<  "unable to close mixer " << snd_strerror(err) << std::endl;
        return;
    }
}

int AlsaRecorder::xrunRecovery(snd_pcm_t *handle, int err)
{
    std::cerr <<  "stream recovery" << std::endl;

    if(err == -EPIPE) {
        /* underrun */
        err = snd_pcm_prepare(handle);
        if(err < 0) {
            std::cerr <<  "Can't recover from underrun, prepare failed: " << snd_strerror(err) << std::endl;
        }
        return 0;
    }
    return err;
}

void AlsaRecorder::destroyAlsa()
{
    std::cerr <<  "ALSA-RX closed." << std::endl;
    // assert(captureHandle != nullptr);
    if(captureHandle != nullptr){
        snd_pcm_drop(captureHandle);
        snd_pcm_close(captureHandle);
        captureHandle = nullptr;
    }
   
}

// vim: set ts=4 sw=4 sts=4 expandtab:
