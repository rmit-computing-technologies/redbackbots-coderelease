/**
 * @author Adhiraj Jain, Kah Hie Toh, Diana Louise Gaba
 */

#include "whistle_detector.h"

#include <iostream>

using namespace std;

int main() {

    std::cout << "Initializing AlsaRecorder" << std::endl;
    AlsaRecorder a(AlsaRecorder::V5_SETTINGS);
    std::cout << "Initializing WhistleDetector" << std::endl;
    WhistleDetector w(a,2300,AlsaRecorder::V5_SETTINGS.sampleRate * 0.4); // audioProvider, minWhistleLength, whistleThreshold  
    std::cout << "Starting Whistle Listening" << std::endl;
    w.start();
    int counter = 0;
    while(true) {
        // Scan for whistle.
        if (w.process()) {
            cout << "Whistle detected. - "<<counter<< endl;
            counter++;
        }
    }
    w.stop();
    return 0;
}

// vim: set ts=4 sw=4 sts=4 expandtab:
