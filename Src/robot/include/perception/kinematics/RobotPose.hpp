/**
 * @file RobotPose.hpp
 * 
 * The RobotPose contains precomputed kinematic data that is useful to other modules.
 * 
 * @author RedbackBots
 */

#pragma once

#include "types/geometry/Point.hpp"
#include "types/geometry/RRCoord.hpp"
#include "types/XYZ_Coord.hpp"
#include "types/camera/CameraInfo.hpp"

#include <Eigen/Dense>
#include <utility>

#include <boost/numeric/ublas/matrix.hpp>

namespace offnao {
   class Motion_Pose;
}

/**
 * The RobotPose class contains precomputed kinematic data that is useful
 * to other modules.
 *
 * The kinematics module computes the full kinematic chain
 * and then stores the resulting matrix in this RobotPose class.
 *
 * Vision then queries the RobotPose class and the cached results are used each time.
 */
class RobotPose {
   public:
      explicit RobotPose(
         Eigen::Matrix4f topCameraToWorldTransform,
         Eigen::Matrix4f botCameraToWorldTransform,
         Eigen::Matrix4f neckToWorldTransform,
         std::pair<int, int> horizon,
         double neckYaw,
         double neckPitch
      );

      RobotPose();

      /**
       *  Returns a pair for the horizon.
       *  pair.first is the y position of the horizon at x = 0
       *  pair.second is the y position of the horizon at x = TOP_IMAGE_COLS
       */
      std::pair<int, int> getHorizon() const;

      XYZ_Coord robotRelativeToNeckCoord(RRCoord coord, int h) const;

      /**
       * Returns the robot relative coord for a given pixel at a particular
       * height.
       * 
       * TODO: This should eventually be removed in favour of the new camera version
       *
       * @param x pixel
       * @param y pixel
       * @param cameraInfo CameraInfo
       * @param h of intersection plane.
       */
      RRCoord imageToRobotRelative(int x, int y, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * Returns the robot relative coord for a given pixel at a particular
       * height.
       *
       * TODO: This should eventually be removed in favour of the new camera version
       * 
       * @param p point in image space
       * @param cameraInfo CameraInfo
       * @param h of intersection plane.
       */
      RRCoord imageToRobotRelative(Point p, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * Returns the robot relative coord for a given point
       * 
       * @param rr_p point in robot relative space
       * @param cameraInfo CameraInfo
       * @param h of intersection plane.
       */
      RRCoord RXYToRobotRelative(Vector2f rr_p, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * NEW VERSION
       * 
       * Returns the Robot Relative X/Y coordinate (on the field plane)
       * of the given point in the camera image, at the given height.
       * 
       * This uses camera properties in the provided Camera Info.
       */
      PointF imageToRobotXY(const PointF &image, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * NEW VERSION
       * 
       * Returns the *CAMERA* Relative X/Y coordinate (on the field plane)
       * of the given point in the camera image, at the given height.
       * 
       * That is, the given point is rotated about Z to the same angle
       * as the camera (head) is pointing. This means the point is interpreted
       * in the rotational alignment of the camera, not the robot torso.
       * 
       * NOTE: For b-human integration this should map to the method: Transformation::imageToRobotWithCameraRotation
       */
      PointF headRelImageToRobotXY(const PointF &image, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * OLD VERSION
       * 
       * Returns the Robot Relative X/Y coordinate (on the field plane)
       * of the given point in the camera image, at the given height.
       * 
       * This uses fixed values of camera settings in CameraDefinitions/VisionDefinitions
       * 
       * TODO: This should eventually be removed in favour of the new camera version.
       * Kept here for use in parts of code yet to be replaced
       */
      Point imageToRobotXY_old(const Point &image, int h = 0) const;

      /**
       * OLD VERSION
       * 
       * "Slow" version of imageToRobotXY used by Offnao
       * TODO: (TW) check if this can be deprecated
       */
      Point imageToRobotXYSlow(const Point &image, int h = 0) const;

      /**
       * NEW VERSION
       * 
       * Returns the Pixel coordinate of the given robot (troso) relative position
       * at the given height in the given camera image.
       */
      Point robotToImageXY(const PointF &image, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * NEW VERSION
       * 
       * Returns the Pixel coordinate of the given robot relative position
       * at the camera (head) rotational angle
       * at the given height in the given camera image.
       * 
       * That is, the given point is first rotated about Z to the same angle
       * as the camera (head) is pointing. This means the point is interpreted
       * in the rotational alignment of the camera, not the robot torso.
       * 
       * NOTE: For b-human integration this should map to the method: Transformation::robotWithCameraRotationToImage
       */
      Point headRelRobotToImageXY(const PointF &robot, const CameraInfo& cameraInfo, int h = 0) const;

      /**
       * NEW VERSION
       * 
       * Alters the Point on plane used in ball detector.
       * Returns a bool indicating if the value was altered.
       */
      bool imageToRobotHorizontalPlane(const Eigen::Vector2f& pointInImage, float z, const CameraInfo& cameraInfo, Eigen::Vector2f& pointOnPlane) const;

      /**
       * OLD VERSION
       * 
       * Returns the Pixel coordinate of the given robot relative position
       * at the given height.
       * 
       * TODO: This should eventually be removed in favour of the new camera version.
       * Kept here for use in parts of code yet to be replaced
       */
      Point robotToImageXY(Point robot, int h = 0) const;

      /* Returns a pointer to the exclusion arrays used by vision */
      // TODO (TW): These were used by Fovea and should "exclude" the body from the camera image
      //            The exclusion arrays are generated in Kinematics.cpp
      //            However, there is no documentation for this method and it seems broken anyway (with 'm' overloaded)
      //            It's best to replace this whole setup with the B-Human BodyContour computation or some equivalent feature
      const int16_t *getTopExclusionArray() const;
      int16_t *getTopExclusionArray();
      const int16_t *getBotExclusionArray() const;
      int16_t *getBotExclusionArray();

      // Returns the camera to world transform for the top or bottom camera
      const Eigen::Matrix4f getC2wTransform(bool top = true) const;

      // OLD C2W TRANSFORM
      const boost::numeric::ublas::matrix<float> getC2wTransformBoost(bool top = true) const;

      // Camera to world transforms
      Eigen::Matrix4f topCameraToWorldTransform;
      Eigen::Matrix4f botCameraToWorldTransform;
      Eigen::Matrix4f topWorldToCameraTransform;
      Eigen::Matrix4f botWorldToCameraTransform;

      // Body (neck) to world transforms as 4x4 matrices.
      Eigen::Matrix4f neckToWorldTransform;
      Eigen::Matrix4f worldToNeckTransform;

      // Camera to neck transforms - for now just storing the joint values
      double neckYaw;
      double neckPitch;

      static const uint16_t EXCLUSION_RESOLUTION = 100;

      // Static field for unchanging camera focal length values. Used by 'toFocus' pre-calcs
      // These are set by naoCameraProvider::setupCamera()
      static float topFocalLength;
      static float botFocalLength;
   
   private:
      Eigen::Vector4f origin;
      Eigen::Vector4f zunit;
      Eigen::Vector4f topCOrigin;
      Eigen::Vector4f botCOrigin;
      Eigen::Vector4f topToFocus;
      Eigen::Vector4f botToFocus;
      Eigen::Matrix4f topWorldToCameraTransformT;
      Eigen::Matrix4f botWorldToCameraTransformT;


      void makeConstants();

      // The horizon is stored as a pair of integers.
      std::pair<int, int> horizon;

      // Exclusion arrays (fixed-size) used by vision
      int16_t topExclusionArray[EXCLUSION_RESOLUTION];
      int16_t botExclusionArray[EXCLUSION_RESOLUTION];

      // Storage for use in calculations - needed to avoid memory reallocation.
      // mutable boost::numeric::ublas::matrix<float> lOrigin;      
      // mutable boost::numeric::ublas::matrix<float> lOrigin2;
      // mutable boost::numeric::ublas::matrix<float> cdir;
      // mutable boost::numeric::ublas::matrix<float> zMatrix;
      // mutable boost::numeric::ublas::matrix<float> rotatedMatrix;

      mutable Eigen::Vector4f lOrigin;
      mutable Eigen::Vector4f lOrigin2;
      mutable Eigen::Vector4f cdir;
      mutable Eigen::Matrix4f zMatrix;
      mutable Eigen::Matrix4f rotatedMatrix;

      /*
       * Internal Image To Robot XY to be given the top/bot values
       */ 
      PointF imageToRobotXY_(const PointF &image, const CameraInfo& cameraInfo, int h,
                             const Eigen::Matrix4f& cameraToWorldTransform,
                             const Eigen::Vector4f& cameraToFocus) const;

      /*
       * Internal Robot To Image XY to be given the top/bot values
       */ 
      // Point robotToImageXY_(const PointF &image, const CameraInfo& cameraInfo, int h,
      //                       const boost::numeric::ublas::matrix<float>& worldToCameraTransform) const;
      Point robotToImageXY_(const PointF &image, const CameraInfo& cameraInfo, int h,
         const Eigen::Matrix4f & worldToCameraTransform) const;

      friend void serialise(const RobotPose &, offnao::Motion_Pose &);
      friend void deserialise(RobotPose &, const offnao::Motion_Pose &);
};

