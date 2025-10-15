/**
 * @file CameraImageDumper.hpp
 * 
 * Only for debugging purposes, dump latest camera images.
*/

#pragma once

#include "utils/debug/DebugDumper.hpp"

#ifdef NDEBUG

    class CameraImageDumper : public DebugDumper {
    public:
        // Create dumper from blackboard configuration
        CameraImageDumper(Blackboard *blackboard)  : DebugDumper(blackboard) {};
        virtual ~CameraImageDumper() {};

        inline void dumpImage(const CameraImage& image) {};
    };

#else

    #include <cstdio>
    #include <string>

    #include "types/camera/CameraImage.hpp"
    #include "utils/Logger.hpp"

    class CameraImageDumper : public DebugDumper {
    public:
        CameraImageDumper(Blackboard *blackboard) :
            DebugDumper(blackboard)
        {
            if (dump) {
                std::string dumpFileName = dir + "/dump_images.yuv";
                llog(INFO) << "Dumping images to: " << dumpFileName << std::endl;
                dumpFile = fopen(dumpFileName.c_str(), "w");
            }
        };
        virtual ~CameraImageDumper() {
            if (dump) {
                fclose(dumpFile);
            }
        };

        // Dump contents of blackboard
        void dumpImage(const CameraImage& image) {
            if (dump) {
                if (dumpTimer.elapsed_us() > dumpRate) {
                    fwrite(image.image, image.width * image.height * sizeof(PixelTypes::YUYVPixel), 1, dumpFile);
                    fflush(dumpFile);
                    dumpTimer.restart();
                }
            }
        };

    private:
        // Dump camera image to file stream
        // TODO (TW): Check if this is the type of input for vatnao
        //            And replace/add latest image dumping as JPEG
        FILE* dumpFile;
    };

#endif
