/**
 * @file LabelImage.hpp
 *
 * Implementation for a module that describes a set of labeling annotations for an image
 *
 * @author Bernd Poppinga
 * @author RedbackBots
 */
#pragma once

#include "types/math/Eigen.hpp"
#include <vector>

struct LabelImage
{
  struct Annotation
  {
    float getIou(const Annotation& annotation) const;
    bool operator<(const Annotation& annotation) const;
    bool isInside(const Annotation& annotation) const;
    float getOverlap(const Annotation& annotation) const;
    float getArea() const;
    Vector2f upperLeft;
    Vector2f lowerRight;
    float confidence;
    bool fallen;
    float distance; // mm
  };

  /**
   * If bounding boxes overlap with an intersection over union score above the threshold,
   * remove all bounding boxes with non-maximal prediction confodence
   * @param threshold
   */
  void nonMaximumSuppression(float threshold = 0.7f);
  
  /**
   * Remove bounding boxes that are inside another bounding box.
   */
  void bigBoxSuppression();

  std::vector<Annotation> annotations;
};
