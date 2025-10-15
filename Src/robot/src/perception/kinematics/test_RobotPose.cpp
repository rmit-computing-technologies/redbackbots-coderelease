#include <iostream>
#include <Eigen/Dense>
#include "perception/kinematics/RobotPose.hpp"

void testGetC2wTransformEigen() {
    Eigen::Matrix4f topMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f botMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f neckMatrix = Eigen::Matrix4f::Identity();
    std::pair<int, int> horizon(0, 0);
    double neckYaw = 0.0;
    double neckPitch = 0.0;

    RobotPose pose(topMatrix, botMatrix, neckMatrix, horizon, neckYaw, neckPitch);

    Eigen::Matrix4f resultTop = pose.getC2wTransform(true);
    Eigen::Matrix4f resultBot = pose.getC2wTransform(false);

    if (resultTop.isApprox(topMatrix) && resultBot.isApprox(botMatrix)) {
        std::cout << "testGetC2wTransformEigen passed." << std::endl;
    } else {
        std::cout << "testGetC2wTransformEigen failed." << std::endl;
    }
}

void testRobotRelativeToNeckCoord() {
    Eigen::Matrix4f topMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f botMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f neckMatrix = Eigen::Matrix4f::Identity();
    std::pair<int, int> horizon(0, 0);
    double neckYaw = 0.0;
    double neckPitch = 0.0;

    RobotPose pose(topMatrix, botMatrix, neckMatrix, horizon, neckYaw, neckPitch);

    RRCoord coord(1.0, 0.0);
    XYZ_Coord result = pose.robotRelativeToNeckCoord(coord, 0);

    if (result.x == 1.0 && result.y == 0.0 && result.z == 0.0) {
        std::cout << "testRobotRelativeToNeckCoord passed." << std::endl;
    } else {
        std::cout << "testRobotRelativeToNeckCoord failed." << std::endl;
    }
}

void testImageToRobotRelative() {
    Eigen::Matrix4f topMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f botMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f neckMatrix = Eigen::Matrix4f::Identity();
    std::pair<int, int> horizon(0, 0);
    double neckYaw = 0.0;
    double neckPitch = 0.0;

    RobotPose pose(topMatrix, botMatrix, neckMatrix, horizon, neckYaw, neckPitch);

    CameraInfo cameraInfo;
    cameraInfo.camera = CameraInfo::Camera::top;
    cameraInfo.opticalCenter = PointF(640, 480);
    cameraInfo.focalLengthInv = 1.0 / 800.0;
    cameraInfo.focalLengthHeightInv = 1.0 / 800.0;

    RRCoord result = pose.imageToRobotRelative(Point(640, 480), cameraInfo, 0);

    if (result.distance() == 0.0 && result.heading() == 0.0) {
        std::cout << "testImageToRobotRelative passed." << std::endl;
    } else {
        std::cout << "testImageToRobotRelative failed." << std::endl;
    }
}

void testRobotToImageXY() {
    Eigen::Matrix4f topMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f botMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f neckMatrix = Eigen::Matrix4f::Identity();
    std::pair<int, int> horizon(0, 0);
    double neckYaw = 0.0;
    double neckPitch = 0.0;

    RobotPose pose(topMatrix, botMatrix, neckMatrix, horizon, neckYaw, neckPitch);

    CameraInfo cameraInfo;
    cameraInfo.camera = CameraInfo::Camera::top;
    cameraInfo.opticalCenter = PointF(640, 480);
    cameraInfo.focalLength = 800.0;
    cameraInfo.focalLengthHeight = 800.0;

    Point result = pose.robotToImageXY(PointF(0.0, 0.0), cameraInfo, 0);

    if (result.x() == 640 && result.y() == 480) {
        std::cout << "testRobotToImageXY passed." << std::endl;
    } else {
        std::cout << "testRobotToImageXY failed." << std::endl;
    }
}

void testHeadRelRobotToImageXY() {
    Eigen::Matrix4f topMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f botMatrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f neckMatrix = Eigen::Matrix4f::Identity();
    std::pair<int, int> horizon(0, 0);
    double neckYaw = 0.0;
    double neckPitch = 0.0;

    RobotPose pose(topMatrix, botMatrix, neckMatrix, horizon, neckYaw, neckPitch);

    CameraInfo cameraInfo;
    cameraInfo.camera = CameraInfo::Camera::top;
    cameraInfo.opticalCenter = PointF(640, 480);
    cameraInfo.focalLength = 800.0;
    cameraInfo.focalLengthHeight = 800.0;

    Point result = pose.headRelRobotToImageXY(PointF(0.0, 0.0), cameraInfo, 0);

    if (result.x() == 640 && result.y() == 480) {
        std::cout << "testHeadRelRobotToImageXY passed." << std::endl;
    } else {
        std::cout << "testHeadRelRobotToImageXY failed." << std::endl;
    }
}