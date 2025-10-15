/**
 * @file ECImageProvider.h
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedBackBots
 */

#pragma once

#include "blackboard/Blackboard.hpp"
#include "perception/vision/Detector.hpp"

#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/ECImage.hpp"

#include <asmjit/asmjit.h>

/**
 * @class ECImageMiddleInfo
 */

class ECImageMiddleInfo : public Detector {
public:
  ECImageMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
  virtual ~ECImageMiddleInfo();

  // Resets middle info
  virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
  virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  virtual void detect_(const VisionInfoIn* info_in, 
                      VisionInfoMiddle* info_middle,
                      VisionInfoOut* info_out);
private:

  void configure(Blackboard* blackboard);

  using EcFunc = void (*)(unsigned int, const void*, void*, void*, void*);
  using EFunc = void (*)(unsigned int, const void*, void*);

  EcFunc ecFunc = nullptr;
  EFunc eFunc = nullptr;
  asmjit::JitRuntime* jitRuntime = nullptr;

  /**
    * Run detection for the given camera
  **/
  void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

  // void detect_(OptionalECImage& theOptionalECImage, const VisionInfoIn* info_in) override;

  void compileE();
  void compileEC();

  /**
   * Extracts single channel chromacity images.
   * As chromacity is only in half resolution in width dimension due to YUYV encoding of the camera,
   * the extracted image get provided in half resolution (width and height).
   * Therefore the values are linearly interpolated in the height dimension.
   * @param eCImage
   */
  void extractChromaticity(const CameraImage* cameraImage, const CameraInfo& cameraInfo, ECImage& eCImage);

  bool disableColor = false;
  bool extractChroma = true;
};
