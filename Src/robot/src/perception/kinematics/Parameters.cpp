#include "perception/kinematics/Parameters.hpp"

template<class T>
Parameters<T>::Parameters() {
   cameraPitchTop    = 0.f;
   cameraYawTop      = 0.f;
   cameraRollTop     = 0.f;
   cameraYawBottom   = 0.f;
   cameraPitchBottom = 0.f;
   cameraRollBottom  = 0.f;
   bodyPitch         = 0.f;
}

template
Parameters<float>::Parameters();

template
Parameters<fadbad::F<float> >::Parameters();

template<> template<>
Parameters<float> Parameters<fadbad::F<float> >::cast<float>()
{
   Parameters<float> casted;

   casted.cameraPitchTop    = cameraPitchTop.x();
   casted.cameraYawTop      = cameraYawTop.x();
   casted.cameraRollTop     = cameraRollTop.x();

   casted.cameraYawBottom   = cameraYawBottom.x();
   casted.cameraPitchBottom = cameraPitchBottom.x();
   casted.cameraRollBottom  = cameraRollBottom.x();

   casted.bodyPitch         = bodyPitch.x();

   return casted;
}

