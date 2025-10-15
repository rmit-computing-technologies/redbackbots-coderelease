/**
 * @file Resize.hpp
 *
 * Functions to resize images.
 *
 * @author Felix Thielke
 * @author RedbackBots
 */

#pragma once

#include "types/Image.hpp"
#include "types/PixelTypes.hpp"

namespace Resize
{
  void shrinkY(const unsigned int downScales, const Image<PixelTypes::GrayscaledPixel>& src, PixelTypes::GrayscaledPixel* dest);

  inline void shrinkY(const unsigned int downScales, const Image<PixelTypes::GrayscaledPixel>& src, Image<PixelTypes::GrayscaledPixel>& dest)
  {
    dest.setResolution(src.width, src.height); // The shrinking algorithm uses more than the final downscaled size.
    dest.setResolution(src.width >> downScales, src.height >> downScales);
    shrinkY(downScales, src, dest[0]);
  }

  void shrinkUV(const unsigned int downScales, const Image<PixelTypes::YUYVPixel>& src, unsigned short* dest);

  inline void shrinkUV(const unsigned int downScales, const Image<PixelTypes::YUYVPixel>& src, Image<unsigned short>& dest)
  {
    dest.setResolution(src.width, src.height); // The shrinking algorithm uses more than the final downscaled size.
    dest.setResolution(src.width >> downScales, src.height >> (downScales + 1));
    shrinkUV(downScales, src, dest[0]);
  }
}
