
/**
 * @file KinematicsPoseTester.cpp
 *
 * Test for Kinematics Pose
 * 
 * @author RedbackBots
 */

#include "perception/vision/other/KinematicsPoseTester.hpp"
#include <iostream>
#include <fstream>
#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/Logger.hpp"
#include <utils/defs/FieldDefinitions.hpp>


#include <iostream>
#include <fstream>
#include <cmath>
#include "perception/kinematics/RobotPose.hpp"
#include "perception/vision/camera/CameraDefinitions.hpp"
#include <thread>

// Creat a global count
int count = 0;

void validatePixelWiseDepth(const RobotPose& robotPose, const CameraInfo& cameraInfo) {
    if (count > 10) {
        // llog(INFO) << NDEBUG_LOGSYMB << "Pixel-wise depth validation already completed." << std::endl;
        return;
    }

    // Experiment settings
    const int CAMERA_TO_TEST = CameraInfo::Camera::top;
    const std::string logFilename = "kinematics_data.csv";

    // Ensure we are checking the correct camera
    // if (cameraInfo.camera != CAMERA_TO_TEST) {
    //     std::cerr << "Error: Invalid camera for pixel-wise depth validation." << std::endl;
    //     return;
    // }

    // Define the expected wall distance and image resolution
    const float EXPECTED_WALL_DISTANCE = 0.20f;
    const int IMAGE_WIDTH = cameraInfo.width;
    const int IMAGE_HEIGHT = cameraInfo.height;

    // Log width and height values
    // llog(INFO) << NDEBUG_LOGSYMB << "Camera width: " << IMAGE_WIDTH << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "Camera height: " << IMAGE_HEIGHT << std::endl;

    // Open log file in truncation mode to ensure overwriting
    std::ofstream logfile(logFilename, std::ios::out | std::ios::trunc);
    if (!logfile.is_open()) {
        std::cerr << "Error: Could not open log file." << std::endl;
        return;
    }

    logfile << "x_pixel, y_pixel, world_x, world_y, error\n";
    int lineCount = 1;

    for (int y = 0; y < IMAGE_HEIGHT; ++y) {
        for (int x = 0; x < IMAGE_WIDTH; ++x) {
            PointF pixelCoord(x, y);
            
            // Compute world coordinates
            // PointF worldCoord = robotPose.imageToRobotXY(pixelCoord, cameraInfo);
            PointF worldCoord = robotPose.headRelImageToRobotXY(pixelCoord, cameraInfo, 0);

            // Compute depth error
            float depthError = std::abs(std::abs(worldCoord.y()) - EXPECTED_WALL_DISTANCE);

            logfile << x << ", " << y << ", "
                    << worldCoord.x() << ", " << worldCoord.y() << ", " 
                    << depthError << "\n";
            
            // Log every 5000th pixel
            // if ((x + y * IMAGE_WIDTH) % 5000 == 0) {
            //     llog(INFO) << NDEBUG_LOGSYMB << "Pixel (" << x << ", " << y << ") -> World ("
            //             << worldCoord.x() << ", " << worldCoord.y() << ") -> Error: " << depthError << std::endl;
            // }

            lineCount++;
        }
    }

    logfile.close();
    // std::cout << "Pixel-wise depth validation completed. Log saved to: " << logFilename << std::endl;

    // // Verify the logfile has expected lines
    // std::cout << "Expected file length: " << (IMAGE_WIDTH * IMAGE_HEIGHT + 1) << " lines." << std::endl;
    // std::cout << "Actual written lines: " << lineCount << " lines." << std::endl;

    count++;
    return;
}


KinematicsPoseTester::KinematicsPoseTester(Blackboard* blackboard):
    Detector("KinematicsPoseTester")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "KinematicsPoseTester loaded" << std::endl;
}

KinematicsPoseTester::~KinematicsPoseTester() {

}

void KinematicsPoseTester::configure(Blackboard* blackboard) {

}

void KinematicsPoseTester::resetMiddleInfo(VisionInfoMiddle* info_middle) {
}

void KinematicsPoseTester::resetVisionOut(VisionInfoOut* info_out) {
}

void KinematicsPoseTester::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Top camera must run first, as the bottom camera is only computed if the top camera boundary can't be projected
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // TODO (TW): decide if lower field boundary is actually needed
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);

    // llog(INFO) << NDEBUG_LOGSYMB << std::endl << std::endl << std::endl << std::endl;
}

void KinematicsPoseTester::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << NDEBUG_LOGSYMB << "- TESTING with " << CameraInfo::enumCameraToString(whichCamera) << std::endl;

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    // llog(INFO) << NDEBUG_LOGSYMB << " - which camera (c) = ("
    //            << cameraInfo.camera << ")"
    //            << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << " - optical center (x,y) = (" 
    //            << cameraInfo.opticalCenter.x() << ","
    //            << cameraInfo.opticalCenter.y() << ")"
    //            << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << " - focal len (len, len height) = (" 
    //            << cameraInfo.focalLength << ","
    //            << cameraInfo.focalLengthHeight << ")"
    //            << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << " - focal len (inv, height inv) = (" 
    //            << cameraInfo.focalLengthInv << ","
    //            << cameraInfo.focalLengthHeightInv << ")"
    //            << std::endl;
    
    // // image -> robot -> image test
    // Point image = Point(cameraInfo.opticalCenter.x(), cameraInfo.height-1);
    // Point image2 = info_in->kinematicPose.robotToImageXY(rxy, cameraInfo, 0);

    // llog(INFO) << NDEBUG_LOGSYMB << "    - ("
    //                    << image.x() << "," << image.y()
    //                    << ")_px = "
    //                    << "("
    //                    << rxy.x() << "," << rxy.y()
    //                    << ")_robot :: "
    //                    << "("
    //                    << image2.x() << "," << image2.y()
    //                    << ")_px2"
    //                    << std::endl;
    // return;

    // PointF image = PointF(-cameraInfo.width/2,0);
    // Point image_i = Point(cameraInfo.opticalCenter.x(),0);

    // PointF rxy1 = PointF(0,0);
    // Point rxy2 = Point(0,0);

    // robot -> image -> robot test
    // for (rxy1.x() = -cameraInfo.width/2; rxy1.x() < cameraInfo.width/2; rxy1.x() += 5) {
    //     for (rxy1.y() = -cameraInfo.height/2; rxy1.y() < cameraInfo.height/2; rxy1.y() += 5) {
    //         Point image = info_in->kinematicPose.robotToImageXY(rxy1, cameraInfo, 0);
    //         PointF rxy2 = info_in->kinematicPose.imageToRobotXY(image.cast<float>(), cameraInfo, 0);
        
    //         llog(INFO) << NDEBUG_LOGSYMB << "    - ("
    //                            << rxy1.x() << "," << rxy1.y()
    //                            << ")_robot = "
    //                            << "("
    //                            << image.x() << "," << image.y()
    //                            << ")_pixel :: "
    //                            << "("
    //                            << rxy2.x() << "," << rxy2.y()
    //                            << ")_back_to_robot"
    //                            << std::endl;
    //     } 
    // }

    PointF rxy1 = PointF(FIELD_WIDTH/2,0);
    PointF rxy2 = PointF(FIELD_WIDTH/2 + CENTER_CIRCLE_DIAMETER/2,0);
    PointF rxy3 = PointF(FIELD_WIDTH/2 - CENTER_CIRCLE_DIAMETER/2,0);
    PointF rxy4 = PointF(FIELD_WIDTH/2 - CENTER_CIRCLE_DIAMETER/2,0);
    PointF rxy5 = PointF(FIELD_WIDTH/2, CENTER_CIRCLE_DIAMETER/2);
    PointF rxy6 = PointF(FIELD_WIDTH/2, -CENTER_CIRCLE_DIAMETER/2);
    PointF rxy7 = PointF(FIELD_WIDTH,0);

    Point image1 = info_in->kinematicPose.robotToImageXY(rxy1, cameraInfo, 0);
    Point image2 = info_in->kinematicPose.robotToImageXY(rxy2, cameraInfo, 0);
    Point image3 = info_in->kinematicPose.robotToImageXY(rxy3, cameraInfo, 0);
    Point image4 = info_in->kinematicPose.robotToImageXY(rxy4, cameraInfo, 0);
    Point image5 = info_in->kinematicPose.robotToImageXY(rxy5, cameraInfo, 0);
    Point image6 = info_in->kinematicPose.robotToImageXY(rxy6, cameraInfo, 0);
    Point image7 = info_in->kinematicPose.robotToImageXY(rxy7, cameraInfo, 0);
    // PointF rxy2 = info_in->kinematicPose.imageToRobotXY(image.cast<float>(), cameraInfo, 0);

    std::vector<Point> gridPoints;
    gridPoints.push_back(image1);
    gridPoints.push_back(image2);
    gridPoints.push_back(image3);
    gridPoints.push_back(image4);
    gridPoints.push_back(image5);
    gridPoints.push_back(image6);
    gridPoints.push_back(image7);
    // info_middle->planeSpots[whichCamera].spots.clear();
    // for (const Point& gridPoint : gridPoints) {
    //         info_middle->planeSpots[whichCamera].spots.push_back(new Spot(gridPoint.x(), gridPoint.y()));
    // }


    // robot -> image -> robot test
    // PointF rxy1 = PointF(1600, 0);
    // Point image = info_in->kinematicPose.robotToImageXY(rxy1, cameraInfo, 0);
    // PointF rxy2 = info_in->kinematicPose.imageToRobotXY(image.cast<float>(), cameraInfo, 0);

    // llog(INFO) << NDEBUG_LOGSYMB << "    - ("
    //                    << rxy1.x() << "," << rxy1.y()
    //                    << ")_px = "
    //                    << "("
    //                    << image.x() << "," << image.y()
    //                    << ")_robot :: "
    //                    << "("
    //                    << rxy2.x() << "," << rxy2.y()
    //                    << ")_px2"
    //                    << std::endl;

    validatePixelWiseDepth(info_in->kinematicPose, cameraInfo);

    // return; /////////////////////////////////////////////////////////////////// remove to run the grid test
    const int GRID_SPACING = 200; // mm
    const int GRID_WIDTH = 3000; // mm
    const int GRID_HEIGHT = 3000; // mm

    // Draw grid lines to an array of spots
    // Cycle through robot relative coordinates in increments of GRID_SPACING and store image points in a vector


    // for (int x = 0; x < GRID_HEIGHT; x += GRID_SPACING) {
    //     for (int y = -GRID_WIDTH/2; y < GRID_WIDTH/2; y += GRID_SPACING) {
    //         PointF rxy = PointF(x, y);
    //         Point imagePoint = info_in->kinematicPose.robotToImageXY(rxy, cameraInfo, 0);
    //         gridPoints.push_back(imagePoint);
    //     }
    // }
    // llog(INFO) << NDEBUG_LOGSYMB << "    - grid points: " << std::endl;
    // for (const Point& gridPoint : gridPoints) {
    //     llog(INFO) << NDEBUG_LOGSYMB << "        - (" << gridPoint.x() << "," << gridPoint.y() << ")" << std::endl;
    // }
    // llog(INFO) << NDEBUG_LOGSYMB << "    - grid points size: " << gridPoints.size() << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "    - grid points (x,y) = (" 
    //            << cameraInfo.opticalCenter.x() << ","
    //            << cameraInfo.opticalCenter.y() << ")"
    //            << std::endl;
    
    info_middle->planeSpots[whichCamera].spots.clear();
    for (const Point& gridPoint : gridPoints) {
        Vector2f imagePoint(gridPoint.x(), gridPoint.y());
        PointF fieldPoint = info_in->kinematicPose.imageToRobotXY(PointF(gridPoint.x(), gridPoint.y()), cameraInfo, 0);
        Vector2f fieldPointVector(fieldPoint.x(), fieldPoint.y());
        info_middle->planeSpots[whichCamera].spots.push_back(new Spot(imagePoint, fieldPointVector));
    }

    return;

    // Take the array of image points and convert them to robot coordinates and back to image points
    // Verify that robot to image and image to robot are perfectly inverse
    // std::vector<Point> camGridPoints;

    // for (const Point& gridPoint : gridPoints) {
    //     PointF rxy = info_in->kinematicPose.imageToRobotXY(gridPoint.cast<float>(), cameraInfo, 0);
    //     Point imagePoint = info_in->kinematicPose.robotToImageXY(rxy, cameraInfo, 0);
        
    //     camGridPoints.push_back(imagePoint);
    // }
    // llog(INFO) << NDEBUG_LOGSYMB << "    - cam grid points: " << std::endl;
    // for (const Point& camGridPoint : camGridPoints) {
    //     llog(INFO) << NDEBUG_LOGSYMB << "        - (" << camGridPoint.x() << "," << camGridPoint.y() << ")" << std::endl;
    // }
    // llog(INFO) << NDEBUG_LOGSYMB << "    - cam grid points size: " << camGridPoints.size() << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "    - cam grid points (x,y) = (" 
    //            << cameraInfo.opticalCenter.x() << ","
    //            << cameraInfo.opticalCenter.y() << ")"
    //            << std::endl;
    // info_middle->camPlaneSpots[whichCamera].spots.clear();
    // for (const Point& camGridPoint : camGridPoints) {
    //         info_middle->camPlaneSpots[whichCamera].spots.push_back(new Spot(camGridPoint.x(), camGridPoint.y()));
    // }

    // return;

    // llog(INFO) << NDEBUG_LOGSYMB << "- Testing PIXEL to ROBOTXY" << std::endl;
    // PointF image = PointF(cameraInfo.opticalCenter.x(),0);
    // Point image_i = Point(cameraInfo.opticalCenter.x(),0);
    // // PointF image = PointF(0,cameraInfo.opticalCenter.y());
    // // Point image_i = Point(0,cameraInfo.opticalCenter.y());
    // PointF rxy = PointF(0,0);
    // Point rxy_old = Point(0,0);
    // // for (image.x() = 0; image.x() != 2; image.x() = image.x() + 1) {
    //     // for (image.y() = 0; image.y() != 2; image.y() = image.y() + 1) {
    // for (image.x() = 0; image.x() < cameraInfo.width; image.x() = image.x() + 5) {
    //     for (image.y() = 0; image.y() < cameraInfo.height; image.y() = image.y() + 5) {
    //         image_i = Point((int) image.x(), (int) image.y());
    //         rxy = info_in->kinematicPose.imageToRobotXY(image, cameraInfo, 0);
    //         rxy_old = info_in->kinematicPose.imageToRobotXY_old(image_i, 0);
    //         llog(INFO) << NDEBUG_LOGSYMB << "    - ("
    //                    << image.x() << "," << image.y()
    //                    << ")_px = "
    //                    << "("
    //                    << rxy.x() << "," << rxy.y()
    //                    << ")_robot :: "
    //                    << "("
    //                    << float(rxy_old.x()) << "," << float(rxy_old.y())
    //                    << ")_old"
    //                    << std::endl;
    //     } 
    // }
    // llog(INFO) << NDEBUG_LOGSYMB << "----- DONE" << std::endl; 

    // // return;

    // Point image_p = Point(cameraInfo.opticalCenter.x(),cameraInfo.opticalCenter.y());
    // // rxy = PointF(-31224,714);
    // rxy = PointF(530,0);
    // Point image_p_old = Point(0,0);
    // // rxy_old = Point(-31224,714);
    // rxy_old = Point(530,0);
    // llog(INFO) << NDEBUG_LOGSYMB << "- Testing ROBOTXY to PIXEL" << std::endl;
    // for (rxy.x() = 0; rxy.x() < 5.0f; rxy.x() = rxy.x() + 0.1) {
    //     for (rxy.y() = 0; rxy.y() < 5.0f; rxy.y() = rxy.y() + 0.1) {
    //         image_p = info_in->kinematicPose.robotToImageXY(rxy, cameraInfo, 0);
    //         image_p_old = info_in->kinematicPose.robotToImageXY(rxy_old, 0);
    //         llog(INFO) << NDEBUG_LOGSYMB << "    - ("
    //                    << rxy.x() << "," << rxy.y()
    //                    << ")_rxy = "
    //                    << "("
    //                    << image_p.x() << "," << image_p.y()
    //                    << ")_px :: "
    //                    << "("
    //                    << image_p_old.x() << "," << image_p_old.y()
    //                    << ")_old"
    //                    << std::endl;
    //     } 
    // }
    // llog(INFO) << NDEBUG_LOGSYMB << "----- DONE" << std::endl; 
    // llog(INFO) << NDEBUG_LOGSYMB << "----------" << std::endl; 
}


