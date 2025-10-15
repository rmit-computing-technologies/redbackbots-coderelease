#include "perception/kinematics/RobotPose.hpp"

#include "soccer.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/camera/CameraDefinitions.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/math/matrix_helpers_eigen.hpp"
#include "utils/Logger.hpp"

#include <cmath>
#include <Eigen/Dense>

// Memory location for statics, and default values
float RobotPose::topFocalLength = DEFAULT_FOCAL_LENGTH;
float RobotPose::botFocalLength = DEFAULT_FOCAL_LENGTH;

RobotPose::RobotPose()
    : lOrigin(Eigen::Vector4f::Zero()),
      lOrigin2(Eigen::Vector4f::Zero()),
      cdir(Eigen::Vector4f::Zero())
{
   for (int i = 0; i < EXCLUSION_RESOLUTION; i++) {
      topExclusionArray[i] = TOP_IMAGE_ROWS;
      botExclusionArray[i] = TOP_IMAGE_ROWS + BOT_IMAGE_ROWS;
   }

   origin = Eigen::Vector4f(0, 0, 0, 1);
   zunit = Eigen::Vector4f(0, 0, 0, 1);
   
   topCameraToWorldTransform  = Eigen::Matrix4f::Identity();
   botCameraToWorldTransform  = Eigen::Matrix4f::Identity();
   topWorldToCameraTransform  = Eigen::Matrix4f::Identity();
   botWorldToCameraTransform  = Eigen::Matrix4f::Identity();
   topWorldToCameraTransformT = Eigen::Matrix4f::Identity();
   botWorldToCameraTransformT = Eigen::Matrix4f::Identity();
   neckToWorldTransform       = Eigen::Matrix4f::Identity();
   worldToNeckTransform       = Eigen::Matrix4f::Identity();
   zMatrix                    = Eigen::Matrix4f::Identity();
   rotatedMatrix              = Eigen::Matrix4f::Identity();
   
   topToFocus = topCameraToWorldTransform * Eigen::Vector4f(0, 0, DEFAULT_FOCAL_LENGTH, 1);
   botToFocus = botCameraToWorldTransform * Eigen::Vector4f(0, 0, DEFAULT_FOCAL_LENGTH, 1);

   horizon = std::pair<int, int>(0, 0);
}

RobotPose::RobotPose(Eigen::Matrix4f topCameraToWorldTransform,
                     Eigen::Matrix4f botCameraToWorldTransform,
                     Eigen::Matrix4f neckToWorldTransform,
                     std::pair<int, int> horizon,
                     double neckYaw,
                     double neckPitch)
   : topWorldToCameraTransform(Eigen::Matrix4f::Zero()),
   botWorldToCameraTransform(Eigen::Matrix4f::Zero()),
   worldToNeckTransform(Eigen::Matrix4f::Zero()),
   neckYaw(neckYaw),
   neckPitch(neckPitch),
   lOrigin(Eigen::Vector4f::Zero()),
   lOrigin2(Eigen::Vector4f::Zero()),
   cdir(Eigen::Vector4f::Zero())
{
   this->topCameraToWorldTransform = topCameraToWorldTransform;
   this->botCameraToWorldTransform = botCameraToWorldTransform;
   this->neckToWorldTransform  = neckToWorldTransform;
   this->horizon  = horizon;

   makeConstants();
}

XYZ_Coord RobotPose::robotRelativeToNeckCoord(RRCoord coord, int h) const {
   Point cartesian = coord.toCartesian();

   Eigen::Vector4f p(cartesian[0], cartesian[1], h, 1);
   Eigen::Vector4f neckP = worldToNeckTransform * p;
   return XYZ_Coord(neckP(0), neckP(1), neckP(2));
}

// TODO: This should eventually be removed in favour of the new camera version
RRCoord RobotPose::imageToRobotRelative(Point p, const CameraInfo& cameraInfo, int h) const {
   PointF rr_p = imageToRobotXY(p.cast<float>(), cameraInfo, h);

   RRCoord r;
   r.distance() = hypotf(rr_p.y(), rr_p.x());
   r.heading()  = atan2f(rr_p.y(), rr_p.x());

   return r;
}

RRCoord RobotPose::RXYToRobotRelative(Vector2f rr_p, const CameraInfo& cameraInfo, int h) const {
   RRCoord r;
   r.distance() = hypotf(rr_p.y(), rr_p.x());
   r.heading()  = atan2f(rr_p.y(), rr_p.x());

   return r;
}

// TODO: This should eventually be removed in favour of the new camera version
RRCoord RobotPose::imageToRobotRelative(int x, int y, const CameraInfo& cameraInfo, int h) const {
   return imageToRobotRelative(Point(x, y), cameraInfo, h);
}

// -------- image To Robot XY implementations

/*
 * NEW VERSION

 * TW Documenting here:
 * new calculation (follow b-human format and use)
 * (opticalcentre.x - image.x) * cameraInfo.focalLengthInv
 * cameraInfo.focalLengthInv is equal to PIXEL (above), but non-fixed values
 * opticalcentre.x is the same as (COLS / 2.0). THis is set in NaoCameraProvider::setupCamera
 *      and uses the .cfg (options) ratio and multiplies by width (COLS)
*/
PointF RobotPose::imageToRobotXY_(const PointF &image, const CameraInfo& cameraInfo, int h,
   const Eigen::Matrix4f& cameraToWorldTransform,
   const Eigen::Vector4f& cameraToFocus) const
{
    // Compute the vector in camera space from the optical center.
    lOrigin2(0) = (cameraInfo.opticalCenter.x() - image.x()) * cameraInfo.focalLengthInv;
    lOrigin2(1) = (cameraInfo.opticalCenter.y() - image.y()) * cameraInfo.focalLengthHeightInv;
    lOrigin2(2) = 0;
    lOrigin2(3) = 1;

    // Apply the camera rotation corrections to the camera-to-world matrix
    // Corrections are applied in camera space (before transformation to world)
   //  Eigen::Matrix4f correctedCameraToWorldTransform = cameraToWorldTransform;
    Eigen::Matrix4f correctedCameraToWorldTransform = cameraToWorldTransform *
        rotateXMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.x) *
        rotateYMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.y) *
        rotateZMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.z);

    // Transform the vector from camera space to world space.
    lOrigin = correctedCameraToWorldTransform * lOrigin2;
 
    // FIXME: Figure out this fkn weird magic number and change it to be per top camera and per bottom camera
    Eigen::Vector4f correctedCameraToFocus = correctedCameraToWorldTransform * 
    Eigen::Vector4f(0, 0, 0.94081740153, 1);

    // Compute the direction vector from the transformed origin to the focus point.
    cdir = correctedCameraToFocus - lOrigin;

    float lambda = (h - lOrigin(2)) / (cdir(2));
    return PointF(lOrigin(0) + lambda * cdir(0),
                  lOrigin(1) + lambda * cdir(1));
    // Compute the vector in camera space from the optical center.

    // llog(DEBUG) << "Image: " << image << std::endl;
    // llog(DEBUG) << "Optical Center: " << cameraInfo.opticalCenter << std::endl;
    // llog(DEBUG) << "Focal Length Inv: " << cameraInfo.focalLengthInv << std::endl;
    // llog(DEBUG) << "Focal Length Height Inv: " << cameraInfo.focalLengthHeightInv << std::endl;

    // llog(DEBUG) << "Calculating vector in camera space from optical center." << std::endl;
    // lOrigin2(0) = (cameraInfo.opticalCenter.x() - image.x()) * cameraInfo.focalLengthInv;
    // lOrigin2(1) = (cameraInfo.opticalCenter.y() - image.y()) * cameraInfo.focalLengthHeightInv;
    // lOrigin2(2) = 0;
    // lOrigin2(3) = 1;
    // llog(DEBUG) << "lOrigin2: " << lOrigin2.transpose() << std::endl;

    // // Apply the camera rotation corrections to the camera matrix
    // llog(DEBUG) << "Applying camera rotation corrections to the camera matrix." << std::endl;
    // Eigen::Matrix4f correctedCameraToWorldTransform = cameraToWorldTransform *
    //     rotateXMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.x).inverse() *
    //     rotateYMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.y).inverse() *
    //     rotateZMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.z).inverse();
    // llog(DEBUG) << "Corrected CameraToWorldTransform: " << correctedCameraToWorldTransform << std::endl;

    // // Create direction vector similar to the other team's approach
    // llog(DEBUG) << "Creating direction vector in world space." << std::endl;
    // Eigen::Vector4f cameraDirection(1.0f, lOrigin2(0), lOrigin2(1), 0);
    // Eigen::Vector4f worldDirection = correctedCameraToWorldTransform * cameraDirection;
    // llog(DEBUG) << "World Direction: " << worldDirection.transpose() << std::endl;

    // // Transform the camera origin to world space
    // llog(DEBUG) << "Transforming camera origin to world space." << std::endl;
    // Eigen::Vector4f cameraOrigin = correctedCameraToWorldTransform * Eigen::Vector4f(0, 0, 0, 1);
    // llog(DEBUG) << "Camera Origin: " << cameraOrigin.transpose() << std::endl;

    // // Ray-plane intersection: find where ray intersects plane at height h
    // llog(DEBUG) << "Calculating ray-plane intersection at height h." << std::endl;
    // float t = (h - cameraOrigin(2)) / worldDirection(2);
    // llog(DEBUG) << "Intersection parameter t: " << t << std::endl;

    // PointF result(cameraOrigin(0) + t * worldDirection(0),
    //               cameraOrigin(1) + t * worldDirection(1));
    // llog(DEBUG) << "Resulting PointF: " << result << std::endl;

    // return result;

}

PointF RobotPose::imageToRobotXY(const PointF &image, const CameraInfo& cameraInfo, int h) const {
    if (cameraInfo.camera == CameraInfo::Camera::top) {
        return imageToRobotXY_(image, cameraInfo, h, topCameraToWorldTransform, topToFocus);
    } else {
        return imageToRobotXY_(image, cameraInfo, h, botCameraToWorldTransform, botToFocus);
    }
    return PointF();
}

/*
 * NEW VERSION
 * 
 * TODO: TW: TEST!!!!
 * NOTE: For b-human integration this should map to the method: Transformation::imageToRobotWithCameraRotation
*/
// TODO: IMPLEMENT!!!!
PointF RobotPose::headRelImageToRobotXY(const PointF &image, const CameraInfo& cameraInfo, int h) const {
   PointF pointF =  imageToRobotXY(image, cameraInfo, h);

   // Bhuman calculation
   //  relative position is the output point
   //  const bool ret = imageToRobot(pointInImage, cameraMatrix, cameraInfo, relativePosition);
   //  Pose3f cameraRotatedMatrix;
   //  cameraRotatedMatrix.rotateZ(cameraMatrix.rotation.getZAngle());
   //  const Vector3f point3D = cameraRotatedMatrix.inverse() * Vector3f(relativePosition.x(), relativePosition.y(), 0.f);
   //  relativePosition = point3D.head<2>();

   // Angle to rotate about
   float zAngle = neckYaw;
   zMatrix = rotateZMatrix<float>(zAngle);

   // llog(DEBUG) << "- zAngle" << zAngle << std::endl;

   // if (cameraInfo.camera == CameraInfo::Camera::top) {
   //    zAngle = std::atan2(topWorldToCameraTransformT(2,1), topWorldToCameraTransformT(1,1));      
   // } else {
   //    zAngle = std::atan2(botWorldToCameraTransformT(2,1), botWorldToCameraTransformT(1,1));
   // }

   // Z-rotated matrix
   // zMatrix = rotateZMatrix<float>(zAngle);

   // This assumes inversion works - which it should since the zMatrix is suitably well-defined
   Eigen::Matrix4f zMatrixInverse = zMatrix.inverse();
   // invertMatrix(zMatrix, zMatrixInverse);

   // Create a homogeneous vector for the point.
   Eigen::Vector4f tmpPoint;
   tmpPoint << pointF.x(), pointF.y(), 0.0f, 1.0f;

    // Rotate the point into the robot-relative coordinate frame.
    Eigen::Vector4f point3D = zMatrixInverse * tmpPoint;
   
    // Update the point with the rotated coordinates.
    pointF.x() = point3D(0);
    pointF.y() = point3D(1);
    return pointF;

   // Rotated version of the camera transform matrix with correct dimensions
   // boost::numeric::ublas::matrix<float> point3D(4,1);

   // Rotate point to robot-relative
   // boost::numeric::ublas::matrix<float> tmpPoint(4,1);
   // tmpPoint(0, 0) = pointF.x();
   // tmpPoint(0, 1) = pointF.y();
   // tmpPoint(0, 2) = 0.0f;
   // noalias(point3D) = prod(zMatrixInverse, tmpPoint);
   
   // // Modify the Point to return with the rotation computation
   // pointF.x() = point3D(0, 0);
   // pointF.y() = point3D(0, 1);

   // return pointF;
}

/*
 * OLD VERSION
 * TODO: Remove eventually
 */
Point RobotPose::imageToRobotXY_old(const Point &image, int h) const {
    // Offnao fix.
    if(offNao) {
        return(imageToRobotXYSlow(image, h));
    }

    // determine which camera
    bool top = true;
    if (image.y() >= 960) {
        top = false;
    }

    // Variations depending on which image we are looking at
    //  const int COLS = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
    //  const int ROWS = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
    const int COLS = (top) ? 1280 : 640;
    const int ROWS = (top) ? 960 : 480;
    const float PIXEL = (top) ? TOP_PIXEL_SIZE : BOT_PIXEL_SIZE;
    
    // PIXEL_TOP = 0.00091854712
    // COLS/2 = 640
    // ROWS/2 = 480

    // calculate vector to pixel in camera space
    lOrigin2(0) = ((COLS / 2.0f - image.x()) * PIXEL);
    lOrigin2(1) = ((ROWS / 2.0f - (image.y() - (!top)*960)) * PIXEL);
    lOrigin2(2) = 0;
    lOrigin2(3) = 1;
    // llog(INFO) << NDEBUG_LOGSYMB << " - lOrigin2 = " 
    //            << lOrigin2
    //            << std::endl;

    if (top) {
      lOrigin = topCameraToWorldTransform * lOrigin2;
      cdir    = topToFocus - lOrigin;
   } else {
         lOrigin = botCameraToWorldTransform * lOrigin2;
         cdir    = botToFocus - lOrigin;
   }

   float lambda = (h - lOrigin(2)) / (cdir(2));
   return Point(lOrigin(0) + lambda * cdir(0),
                lOrigin(1) + lambda * cdir(1));
}

/* OLD VERSION
 * TODO: Remove eventually
 * TODO: (TW) Determine if this is exactly the same as the "non-slow"
*/
Point RobotPose::imageToRobotXYSlow(const Point &image, int h) const {
    // determine which camera
    bool top = true;
    if (image.y() >= TOP_IMAGE_ROWS) {
        top = false;
    }

    // Variations depending on which image we are looking at
    const int COLS = (top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
    const int ROWS = (top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;
    const float PIXEL = (top) ? TOP_PIXEL_SIZE : BOT_PIXEL_SIZE;

    // calculate vector to pixel in camera space
    lOrigin2(0) = ((COLS / 2.0f - image.x()) * PIXEL);
    lOrigin2(1) = ((ROWS / 2.0f - (image.y() - (!top)*TOP_IMAGE_ROWS)) * PIXEL);
    lOrigin2(2) = 0;
    lOrigin2(3) = 1;

    if (top) {
      lOrigin = topCameraToWorldTransform * lOrigin2;
      cdir    = topToFocus - lOrigin;
   } else {
         lOrigin = botCameraToWorldTransform * lOrigin2;
         cdir    = botToFocus - lOrigin;
   }

   float lambda = (h - lOrigin(2)) / (cdir(2));
   return Point(lOrigin(0) + lambda * cdir(0),
                  lOrigin(1) + lambda * cdir(1));
}

// -------- robot To image XY implementations

/*
 * NEW VERSION

 * TODO (TW): THIS REQUIRES TESTING!!!!!
 * This should be the inverse of the computation in imageToRobotXY.
 * There the pixel computation is:
 * lOrigin2(0, 0) = (cameraInfo.opticalCenter.x() - image.x()) * cameraInfo.focalLengthInv
 * lOrigin2(1, 0) = (cameraInfo.opticalCenter.y() - image.y()) * cameraInfo.focalLengthHeightInv
*/
Point RobotPose::robotToImageXY_(const PointF &robot, const CameraInfo& cameraInfo, int h,
   const Eigen::Matrix4f& worldToCameraTransform) const
{
   // llog(INFO) << NDEBUG_LOGSYMB << " - robot = " << robot << std::endl;

   // Apply the camera rotation corrections to the camera matrix
   Eigen::Matrix4f cameraToWorldWithCorrections = 
      rotateXMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.x) *
      rotateYMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.y) *
      rotateZMatrix<float>(cameraInfo.cameraCalibrations.cameraRotationCorrections.z);

   Eigen::Matrix4f correctedWorldToCameraTransform = 
      cameraToWorldWithCorrections.inverse() * worldToCameraTransform;


   // llog(INFO) << NDEBUG_LOGSYMB << " - camera rotation corrections (radians) = "
   //            << cameraInfo.cameraCalibrations.cameraRotationCorrections.x << ", "
   //            << cameraInfo.cameraCalibrations.cameraRotationCorrections.y << ", "
   //            << cameraInfo.cameraCalibrations.cameraRotationCorrections.z
   //            << std::endl;

   // llog(INFO) << NDEBUG_LOGSYMB << " - correctedWorldToCameraTransform = " 
   //            << correctedWorldToCameraTransform << std::endl;

   Eigen::Vector4f p(robot.x(), robot.y(), h, 1);
   // llog(INFO) << NDEBUG_LOGSYMB << " - p = " << p << std::endl;

   Eigen::Vector4f pixel = correctedWorldToCameraTransform * p;
   // llog(INFO) << NDEBUG_LOGSYMB << " - pixel pre normalization = " << pixel << std::endl;

   // Normalize by the homogeneous coordinate.
   pixel(0) /= std::abs(pixel(3));
   pixel(1) /= std::abs(pixel(3));
   // llog(INFO) << NDEBUG_LOGSYMB << " - pixel post normalization = " << pixel << std::endl;

   // Map to pixel coordinates using the camera intrinsics.
   pixel(0) = cameraInfo.opticalCenter.x() + (pixel(0) * cameraInfo.focalLength);
   pixel(1) = cameraInfo.opticalCenter.y() + (pixel(1) * cameraInfo.focalLengthHeight);

   // llog(INFO) << NDEBUG_LOGSYMB << " - OpticalCenter = "
   //            << cameraInfo.opticalCenter.x() << ", " << cameraInfo.opticalCenter.y()
   //            << std::endl;

   // llog(INFO) << NDEBUG_LOGSYMB << " - focalLength px = "
   //            << cameraInfo.focalLength
   //            << std::endl;

   // llog(INFO) << NDEBUG_LOGSYMB << " - focalLengthHeight px = "
   //            << cameraInfo.focalLengthHeight
   //            << std::endl;

   // llog(INFO) << NDEBUG_LOGSYMB << " - pixel final = " << pixel(0) << ", " << pixel(1) << std::endl;

   return Point(pixel(0), pixel(1));
}

Point RobotPose::robotToImageXY(const PointF &robot, const CameraInfo& cameraInfo, int h) const {
   if (cameraInfo.camera == CameraInfo::Camera::top) {
      return robotToImageXY_(robot, cameraInfo, h, topWorldToCameraTransformT);
   } else {
      return robotToImageXY_(robot, cameraInfo, h, botWorldToCameraTransformT);
   }
   return Point();
}

/*
 * NEW VERSION

 * Head relative - that is, transform a point to camera relative coordinates

 * NOTE: For b-human integration this should map to the method: Transformation::robotWithCameraRotationToImage
*/
Point RobotPose::headRelRobotToImageXY(const PointF &robot, const CameraInfo& cameraInfo, int h) const {
   float zAngle = neckYaw;
   zMatrix = rotateZMatrix<float>(zAngle);
   
   if (cameraInfo.camera == CameraInfo::Camera::top) {
      rotatedMatrix = topWorldToCameraTransformT * zMatrix;
   } else {
      rotatedMatrix = botWorldToCameraTransformT * zMatrix;
   }
   
   return robotToImageXY_(robot, cameraInfo, h, rotatedMatrix);
}


// -------- additional methods/helpers

std::pair<int, int> RobotPose::getHorizon() const {
   return horizon;
}

const int16_t *RobotPose::getTopExclusionArray() const {
   return topExclusionArray;
}

int16_t *RobotPose::getTopExclusionArray() {
   return topExclusionArray;
}

const int16_t *RobotPose::getBotExclusionArray() const {
   return botExclusionArray;
}

int16_t *RobotPose::getBotExclusionArray() {
   return botExclusionArray;
}

const Eigen::Matrix4f RobotPose::getC2wTransform(bool top) const {
   return (top ? topCameraToWorldTransform : botCameraToWorldTransform);
}

void RobotPose::makeConstants() {
   Eigen::Matrix4f projection;  
   projection << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, CAMERA_FOV_W, 0;
               //   0, 0, tan(CAMERA_FOV_W / 2), 0;

   // Invert the camera-to-world transforms to obtain world-to-camera transforms.
   topWorldToCameraTransform = topCameraToWorldTransform.inverse();
   topWorldToCameraTransformT = projection * topWorldToCameraTransform;
   botWorldToCameraTransform = botCameraToWorldTransform.inverse();
   botWorldToCameraTransformT = projection * botWorldToCameraTransform;

   worldToNeckTransform = neckToWorldTransform.inverse();

   // Set the origin and compute the camera origins.
   origin   = Eigen::Vector4f(0, 0, 0, 1);
   zunit    = Eigen::Vector4f(0, 0, 0, 1);
   topCOrigin = topCameraToWorldTransform * origin;
   botCOrigin = botCameraToWorldTransform * origin;

   // Compute the focal points using a “hackjob” focal length.
   topToFocus = topCameraToWorldTransform * Eigen::Vector4f(0, 0, 0.94081740153 , 1);
   botToFocus = botCameraToWorldTransform * Eigen::Vector4f(0, 0, 0.94081740153 , 1);

}


/////////////////// OLD IMPLEMENTATIONS USING BOOST ///////////////////
// TODO (BK): Kept here for use in other areas of code. should be eventually removed. 
const boost::numeric::ublas::matrix<float>
      RobotPose::getC2wTransformBoost(bool top) const {
   boost::numeric::ublas::matrix<float> c(4, 4);
   if (top) {
      c(0, 0) = topCameraToWorldTransform(0, 0);
      c(1, 0) = topCameraToWorldTransform(1, 0);
      c(2, 0) = topCameraToWorldTransform(2, 0);
      c(3, 0) = topCameraToWorldTransform(3, 0);
      c(0, 1) = topCameraToWorldTransform(0, 1);
      c(1, 1) = topCameraToWorldTransform(1, 1);
      c(2, 1) = topCameraToWorldTransform(2, 1);
      c(3, 1) = topCameraToWorldTransform(3, 1);
      c(0, 2) = topCameraToWorldTransform(0, 2);
      c(1, 2) = topCameraToWorldTransform(1, 2);
      c(2, 2) = topCameraToWorldTransform(2, 2);
      c(3, 2) = topCameraToWorldTransform(3, 2);
      c(0, 3) = topCameraToWorldTransform(0, 3);
      c(1, 3) = topCameraToWorldTransform(1, 3);
      c(2, 3) = topCameraToWorldTransform(2, 3);
      c(3, 3) = topCameraToWorldTransform(3, 3);
   } else {
      c(0, 0) = botCameraToWorldTransform(0, 0);
      c(1, 0) = botCameraToWorldTransform(1, 0);
      c(2, 0) = botCameraToWorldTransform(2, 0);
      c(3, 0) = botCameraToWorldTransform(3, 0);
      c(0, 1) = botCameraToWorldTransform(0, 1);
      c(1, 1) = botCameraToWorldTransform(1, 1);
      c(2, 1) = botCameraToWorldTransform(2, 1);
      c(3, 1) = botCameraToWorldTransform(3, 1);
      c(0, 2) = botCameraToWorldTransform(0, 2);
      c(1, 2) = botCameraToWorldTransform(1, 2);
      c(2, 2) = botCameraToWorldTransform(2, 2);
      c(3, 2) = botCameraToWorldTransform(3, 2);
      c(0, 3) = botCameraToWorldTransform(0, 3);
      c(1, 3) = botCameraToWorldTransform(1, 3);
      c(2, 3) = botCameraToWorldTransform(2, 3);
      c(3, 3) = botCameraToWorldTransform(3, 3);
   }
   return c;
}
