#pragma once

#include "FADBAD++/fadiff.h"

template <class T>
class Parameters {
   public:
      Parameters();

      T cameraPitchTop;
      T cameraYawTop;
      T cameraRollTop;

      T cameraYawBottom;
      T cameraPitchBottom;
      T cameraRollBottom;

      T bodyPitch;

      /* Used only to convert fadbad::F<float> to float.
       * Needed by KinematicsCalibrationSkill
       */
      template<typename T_>
      Parameters<T_> cast();

};

