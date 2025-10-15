/**
 * @file IISC.cpp
 *
 * Methods to help validate perception.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
 */

#include "utils/math/IISC.hpp"
#include "utils/SpatialUtilities.hpp"

bool IISC::calculateImagePenaltyMeasurementsByCenter(const Vector2f& center, float& length, float& height, const VisionInfoIn* info_in, const CameraInfo& cameraInfo)
{
  height = getImageDiameterByLowestPointAndFieldDiameter(PENALTY_CROSS_DIMENSIONS, center, info_in, cameraInfo);
  length = getHorizontalImageDiameterByMiddlePointAndFieldDiameter(PENALTY_CROSS_DIMENSIONS, center, info_in, cameraInfo);
  return height >= 0.f && length >= 0.f;
}

float IISC::getImageDiameterByLowestPointAndFieldDiameter(const float fieldDiameter, const Vector2f& start, const VisionInfoIn* info_in, const CameraInfo& cameraInfo)
{
    //if(!Transformation::imageToRobot(start, theCameraMatrix, theCameraInfo, point))
    Vector2f point = info_in->kinematicPose.imageToRobotXY(start, cameraInfo);
    if (!SpatialUtilities::possiblyOnFieldRRXY(point)) {
        return -1.f;
    }

    point.normalize(point.norm() + fieldDiameter);

    //if(!Transformation::robotToImage(point, theCameraMatrix, theCameraInfo, newImagePoint))
    Vector2f newImagePoint = info_in->kinematicPose.robotToImageXY(point, cameraInfo).cast<float>();
    if (newImagePoint.x() < 0) {
        return -1.f;
    }

    return (newImagePoint - start).norm();
}

float IISC::getHorizontalImageDiameterByMiddlePointAndFieldDiameter(const float fieldDiameter, const Vector2f& middle, const VisionInfoIn* info_in, const CameraInfo& cameraInfo)
{
    //if(!Transformation::imageToRobot(middle, theCameraMatrix, theCameraInfo, point))
    Vector2f point = info_in->kinematicPose.imageToRobotXY(middle, cameraInfo);
    if (!SpatialUtilities::possiblyOnFieldRRXY(point)) {
        return -1.f;
    }

    point += point.normalized(fieldDiameter / 2.f).rotateLeft();

    //if(!Transformation::robotToImage(point, theCameraMatrix, theCameraInfo, newImagePoint))
    Vector2f newImagePoint = info_in->kinematicPose.robotToImageXY(point, cameraInfo).cast<float>();
    if (newImagePoint.x() < 0) {
        return -1.f;
    }

    return 2.f * (newImagePoint - middle).norm();
}

bool IISC::calcPossibleVisibleBallByLowestPoint(const Vector2f& start, Geometry::Circle& circle, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, const Angle greenEdge)
{
    Vector2f startPoint = info_in->kinematicPose.imageToRobotXY(start, cameraInfo);
    //if(!Transformation::imageToRobot(start, theCameraMatix, theCameraInfo, startPoint))
    if (!SpatialUtilities::possiblyOnFieldRRXY(startPoint)) {
        return false;
    }

    const Vector2f slightlyRightPoint(start.x() + 1.f, start.y());
    Vector2f slightlyRightField = info_in->kinematicPose.imageToRobotXY(slightlyRightPoint, cameraInfo);
    //if(!Transformation::imageToRobot(slightlyRightPoint, theCameraMatix, theCameraInfo, slightlyRightField))
    if (!SpatialUtilities::possiblyOnFieldRRXY(slightlyRightField)) {
        return false;
    }

    const Vector3f centerPoint3f(startPoint.x(), startPoint.y(), BALL_RADIUS);
    //const Vector3f cameraPointVector(centerPoint3f - theCameraMatix.translation);
    const Vector3f slightlyRightField3f(slightlyRightField.x(), slightlyRightField.y(), BALL_RADIUS);
    const Vector3f rightVector(slightlyRightField3f - centerPoint3f);

    const Vector3f dir(rightVector.cross(centerPoint3f)); // change to cameraPointVector

    const Angle dirAngle(std::asin(dir.normalized(BALL_RADIUS).z() / BALL_RADIUS));

    const Angle rotateAngle(greenEdge + dirAngle < (M_PI / 2.f) ? 0_rad : Angle(greenEdge + dirAngle - (M_PI / 2.f)));
    const float visiblePercentage = 0.5f + 0.5f * std::cos(rotateAngle);
    const AngleAxisf rotationAxis(-rotateAngle, Vector3f(dir.y(), dir.x(), 0.f).normalized());
    const Vector3f useDir(RotationMatrix(rotationAxis) * dir.normalized(BALL_RADIUS));

    const Vector3f ballRayIntersectionOffset(-useDir - Vector3f(0.f, 0.f, -BALL_RADIUS));

    Vector2f ballPointByStart = info_in->kinematicPose.imageToRobotXY(start, cameraInfo, ballRayIntersectionOffset.z());
    //if(!Transformation::imageToRobotHorizontalPlane(start, ballRayIntersectionOffset.z(), theCameraMatix, theCameraInfo, ballPointByStart))
    if (!SpatialUtilities::possiblyOnFieldRRXY(ballPointByStart)) {
        return false;
    }

    const Vector3f ballPointByStart3f(ballPointByStart.x(), ballPointByStart.y(), ballRayIntersectionOffset.z());
    const Vector3f highesVisibleBallPoint(ballPointByStart3f + dir.normalized((2.f * BALL_RADIUS) * visiblePercentage));
    const Vector2f highesVisibleBallPoint2f = Vector2f(highesVisibleBallPoint.x(), highesVisibleBallPoint.y());

    Vector2f highesImagePoint = info_in->kinematicPose.robotToImageXY(highesVisibleBallPoint2f, cameraInfo, highesVisibleBallPoint.z()).cast<float>();
    //if(!Transformation::robotToImage(highesVisibleBallPoint, theCameraMatix, theCameraInfo, highesImagePoint))
    if (highesImagePoint.x() < 0) {
        return false;
    }

    const float visibleDiameter = (start - highesImagePoint).norm();

    circle.radius = (visibleDiameter / visiblePercentage) / 2.f;
    circle.center = highesImagePoint + (start - highesImagePoint).normalized(circle.radius);

    return true;
}