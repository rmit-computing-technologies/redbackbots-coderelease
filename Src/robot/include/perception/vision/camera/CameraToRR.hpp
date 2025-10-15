#pragma once

#include <stdint.h>
#include <math.h>

#include "perception/vision/VisionDefinitions.hpp"
#include "perception/kinematics/RobotPose.hpp"
#include "types/RansacTypes.hpp"
#include "types/SensorValues.hpp"
#include "types/geometry/Point.hpp"
#include "types/geometry/RRCoord.hpp"



/**
 * CameraToRR
 *
 * Functions for generating "Robot Relative" measurements and conversions.
 *
 * All constant measurements are in mm or radians
 **/

class CameraToRR {
   public:
      CameraToRR();
      ~CameraToRR();

      SensorValues values;
      RobotPose pose;

      // Decide if it's worthwhile respecting "private data" for values
      void updateAngles(SensorValues values);
      
      RRCoord convertToRR(int16_t i, int16_t j, const CameraInfo& cameraInfo, bool isBall) const;
      RRCoord convertToRR(const Point &p, const CameraInfo& cameraInfo, bool isBall) const;
      Point convertToRRXY(const Point &p) const;
      RANSACLine convertToRRLine(const RANSACLine &l) const;
      Point convertToImageXY(const Point &p) const;

      float pixelSeparationToDistance(int pixelSeparation, int realSeparation) const;
      bool isRobotMoving() const;

      // TODO (TW): REMOVE : scan lines - only used by fovea generation

      /**
       * Finds the saliency scan coordinates where vertical scans
       * should be stopped to avoid considering the robots own body
       * The coordinates in the array returned are image co-ords
       **/
//       void findEndScanValues();

      int getTopEndScanCoord(int index) const {
            return topEndScanCoords_[index];
      }
      int getBotEndScanCoord(int index) const {
            return botEndScanCoords_[index];
      }

    private:
      int topEndScanCoords_[TOP_IMAGE_COLS];
      int botEndScanCoords_[BOT_IMAGE_COLS];
};
