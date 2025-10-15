/**
 * @file KeypointsDetector.cpp
 *
 * This file implements a module that subsamples a centered patch from the
 * camera image and uses MoveNet to detect keypoints in it. The keypoints
 * correspond to different body parts of a single person.
 *
 * @author Thomas Röfer
 * @author RedbackBots
*/

#include "perception/vision/referee/KeypointsDetector.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/debug/Assert.hpp"
#include "types/ColorModelConversions.hpp"

#include <asmjit/asmjit.h>

KeypointsDetector::KeypointsDetector(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
    Detector("KeypointsDetector"),
    detector(jitRuntime),
    kpModel(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string keypointFile = modelDir + "/Keypoints/movenet_singlepose_thunder_3.onnx";

    llog(INFO) << NDEBUG_LOGSYMB << "Keypoint Detector Loading keypoints Model from file " << keypointFile << std::endl;

    kpModel = std::make_unique<NeuralNetworkONNX::Model>(keypointFile);

    detector.compile(*kpModel);

    llog(INFO) << NDEBUG_LOGSYMB << "Keypoint detector Models loaded and compiled" << std::endl;
}

KeypointsDetector::~KeypointsDetector() {
    kpModel = nullptr;
    blackboard = nullptr;
}

void KeypointsDetector::configure(Blackboard* blackboard) {
    this->blackboard = blackboard;
}

void KeypointsDetector::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->refereeKeypoints.reset();
}

void KeypointsDetector::resetVisionOut(VisionInfoOut* info_out) {

}

void KeypointsDetector::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void KeypointsDetector::detect_(CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {  
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    const CameraImage* cameraImage = info_in->image[whichCamera];

    RefereeKeypoints& refereeKeypoints = info_middle->refereeKeypoints;

    const unsigned centerX = cameraImage->width;
    const unsigned centerY = patchAtTop ? patchSize / 2 : cameraImage->height / 2;

    const CameraImage& refCameraImage = *cameraImage; // PREVENTS SEGMENT FAULTS

    Vector2f robotPos = readFrom(stateEstimation, robotPos).convertToVector();
    if(mask.empty() || (robotPos - lastRobotPosition).norm() > maskRecomputeThreshold)
    {
        createMask(centerX, centerY, patchSize, patchSize, detector.input(0).dims(0), detector.input(0).dims(1), refCameraImage, blackboard);
        lastRobotPosition = robotPos;
    }

    ASSERT(detector.input(0).dims(0) == detector.input(0).dims(1));
    ASSERT(detector.input(0).dims(2) == 3);

    // The only patch size supported.
    const int height = patchSize;
    const int width = patchSize;

    refereeKeypoints.patchBoundary = Boundaryi(Rangei(centerX - width / 2, centerX + width / 2),
                                            Rangei(centerY - height / 2, centerY + height / 2));

    if(detector.input(0).dims(0) == 192 && patchSize == 384) {
        extractPatch2to1(centerX, centerY, height, width, detector.input(0).data(), refCameraImage);
    }
    else if(detector.input(0).dims(0) == 256 && patchSize == 384) {
        extractPatch3to2(centerX, centerY, height, width, detector.input(0).data(), refCameraImage);
    }
    else if(detector.input(0).dims(0) == static_cast<unsigned>(patchSize)) {
        extractPatch1to1(centerX, centerY, height, width, detector.input(0).data(), refCameraImage);
    }

    applyMask(detector.input(0).dims(0), detector.input(0).dims(1), detector.input(0).data()); // TODO: Comment out if testing without mask

    detector.apply();

    const Output* outputs = reinterpret_cast<Output*>(detector.output(0).data());

    for(int i = 0; i != RefereeKeypoints::Keypoint::count; i++) {
        const Output& output = outputs[i];
        RefereeKeypoints::Point& point = refereeKeypoints.points[i];
        point.position = Vector2f(centerX + (output.x - 0.5f) * width,
                                centerY + (output.y - 0.5f) * height);
        point.valid = output.confidence >= minConfidence;
    }
}

void KeypointsDetector::createMask(const unsigned centerX, const unsigned centerY, const int height, const int width, const int patchHeight, const int patchWidth,  
    const CameraImage& cameraImage, Blackboard *blackboard) {

    const Vector2f refereeOnField(0, (((FIELD_WIDTH / 2) + (FIELD_WIDTH / 2 + FIELD_WIDTH_OFFSET)) / 2.f) * (readFrom(gameController, leftTeam) ? 1 : -1));
    const Vector2f refereeOffset = refereeOnField - readFrom(stateEstimation, robotPos).convertToVector();
    const float yScale = 3000.f / refereeOffset.norm();
    const float xScale = std::cos(std::abs(refereeOffset.angle()) - 90_deg);

    mask.clear();
    const Geometry::Circle circle({static_cast<float>(centerX),
    static_cast<float>(centerY) + (maskCenterY - static_cast<float>(cameraImage.height / 2)) * yScale}, maskRadius * yScale);
    const int maskWidth = std::min(width, static_cast<int>(this->maskWidth * xScale * yScale));
    for(int patchY = 0; patchY < patchHeight; ++patchY) {
        const int y = centerY - height / 2 + patchY * height / patchHeight;
        const Geometry::Line row(Vector2i(0, y), Vector2i(1, 0));
        Vector2f inter1, inter2;
        Rangei range(centerX - maskWidth / 2, centerX + maskWidth / 2);
        if(Geometry::getIntersectionOfLineAndCircle(row, circle, inter1, inter2) == 2) {
            range.min = std::max(static_cast<int>(centerX) - width / 2,
            std::min(range.min, static_cast<int>(centerX) + static_cast<int>((std::min(inter1.x(), inter2.x()) - static_cast<float>(centerX)) * xScale)));
            range.max = std::min(static_cast<int>(centerX) + width / 2,
            std::max(range.max, static_cast<int>(centerX) + static_cast<int>((std::max(inter1.x(), inter2.x()) -
            static_cast<float>(centerX)) * xScale) + 1));
        }
        mask.emplace_back((range.min - centerX + width / 2) * patchWidth / width,
        (range.max - centerX + width / 2) * patchWidth / width);
    }
}

void KeypointsDetector::applyMask(const int patchHeight, const int patchWidth, float* data) const {
    for(int y = 0; y < patchHeight; ++y) {
        const Rangei& range = mask[y];
        float* row = data + y * patchWidth * 3;
        for(float* p = row, *pEnd = row + range.min * 3; p < pEnd;) {
            *p++ = 255.f;
            *p++ = 0.f;
            *p++ = 255.f;
        }
        for(float* p = row + range.max * 3, *pEnd = row + patchWidth * 3; p < pEnd;) {
            *p++ = 255.f;
            *p++ = 0.f;
            *p++ = 255.f;
        }
    }
}

void KeypointsDetector::extractPatch1to1(const unsigned centerX, const unsigned centerY, const int height, const int width, float* channel, 
 const CameraImage& cameraImage) const {
    for(unsigned y = centerY - height / 2; y < centerY + height / 2; ++y) {
        for(const CameraImage::PixelType* pixel = &cameraImage[y][centerX / 2 - width / 4], *pixelEnd = pixel + width / 2; pixel < pixelEnd; ++pixel) {
            const CameraImage::PixelType yuyv = *pixel;
            unsigned char r, g, b;
            ColorModelConversions::fromYUVToRGB(yuyv.y0, yuyv.u, yuyv.v, r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ColorModelConversions::fromYUVToRGB(yuyv.y1, yuyv.u, yuyv.v, r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
        }
    }
}

void KeypointsDetector::extractPatch2to1(const unsigned centerX, const unsigned centerY, const int height, const int width, float* channel, 
 const CameraImage& cameraImage) const {
    for(unsigned y = centerY - height / 2; y < centerY + height / 2; y += 2) {
        for(const CameraImage::PixelType* pixel = &cameraImage[y][centerX / 2 - width / 4], *pixelEnd = pixel + width / 2; pixel < pixelEnd; ++pixel) {
            unsigned char r, g, b;
            ColorModelConversions::fromYUVToRGB(pixel->y1, pixel->u, pixel->v, r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
        }
    }
}

void KeypointsDetector::extractPatch3to2(const unsigned centerX, const unsigned centerY, const int height, const int width, float* channel, 
 const CameraImage& cameraImage) const {
    for(unsigned y = centerY - height / 2; y < centerY + height / 2; y += 3) {
        for(const CameraImage::PixelType* pixel = &cameraImage[y][centerX / 2 - width / 4], * pixelEnd = pixel + width / 2; pixel < pixelEnd;) {
            unsigned char r, g, b;
            ColorModelConversions::fromYUVToRGB(pixel->y0, pixel->u, pixel->v, r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y1) + pixel[1].y0) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel[1].u) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel[1].v) >> 1),
                r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ++pixel;
            ColorModelConversions::fromYUVToRGB(pixel->y1, pixel->u, pixel->v, r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ++pixel;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y0) + pixel->y1) >> 1),
                pixel->u, pixel->v, r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ++pixel;
        }
        for(const CameraImage::PixelType* pixel = &cameraImage[y + 1][centerX / 2 - width / 4],
         * pixel2 =  &cameraImage[y + 2][centerX / 2 - width / 4],
         * pixelEnd = pixel + width / 2; pixel < pixelEnd;) {
            unsigned char r, g, b;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y0) + pixel2->y0) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel2->u) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel2->v) >> 1),
                r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y1) + pixel[1].y0 + pixel2->y1 + pixel2[1].y0) >> 2),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel[1].u + pixel2->u + pixel2[1].u) >> 2),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel[1].v + pixel2->v + pixel2[1].v) >> 2),
                r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ++pixel;
            ++pixel2;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y1) + pixel2->y1) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel2->u) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel2->v) >> 1),
                r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ++pixel;
            ++pixel2;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>((static_cast<unsigned short>(pixel->y0) + pixel->y1 + pixel2->y0 + pixel2->y1) >> 2),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->u) + pixel2->u) >> 1),
                static_cast<unsigned char>((static_cast<unsigned short>(pixel->v) + pixel2->v) >> 1),
                r, g, b);
            *channel++ = r;
            *channel++ = g;
            *channel++ = b;
            ++pixel;
            ++pixel2;
        }
    }
}
