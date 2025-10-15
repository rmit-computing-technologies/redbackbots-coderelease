/**
 * @file CameraImage.h
 *
 * Declares a representation that allows for using the CameraImage as a Image<YUYVPixel>.
 *
 * @author Felix Thielke
 * @author RedbackBots
 */

#pragma once


#include "types/Image.hpp"
#include "types/PixelTypes.hpp"


/**
 * IMPORANT: that the image uses a YUYV (2-pixel wide) format in the width (see template type) with Chroms Subsampling.
 * That is, each 'pixel' in YUYV is 2 actual pixels.
 * Therefore the 'internal width' of the image is 'divided by 2', which is then multiplied by 2 with the YUYV Pixel Type.
 * So when iterating over this Camera Image, you need to 'multiple' the width by 2 to  get the 'true' image width
 * See: https://en.wikipedia.org/wiki/Chroma_subsampling
*/
struct CameraImage : public Image<PixelTypes::YUYVPixel> {
private:

public:
    unsigned int timestamp = 0;

    static constexpr unsigned int maxResolutionWidth = 1280;
    static constexpr unsigned int maxResolutionHeight = 960;

    void setImage(const unsigned int width, const unsigned int height, void* data, const unsigned int timestamp = 0) {
        this->width = width;
        this->height = height;
        this->timestamp = timestamp;
        image = reinterpret_cast<PixelType*>(data);
    }

    unsigned char getY(const size_t x, const size_t y) const {
        return *(reinterpret_cast<const unsigned char*>(image) + y * width * 4 + x * 2);
    }
    uint8_t getU(const size_t x, const size_t y) const { 
        return (*this)(x / 2, y).u; 
    }
    uint8_t getV(const size_t x, const size_t y) const { 
        return (*this)(x / 2, y).v; 
    }

    PixelTypes::YUVPixel getYUV(const size_t x, const size_t y) const {
        const PixelTypes::YUYVPixel& yuyv = (*this)(x / 2, y);
        PixelTypes::YUVPixel yuv;
        yuv.y = yuyv.y(x);
        yuv.u = yuyv.u;
        yuv.v = yuyv.v;
        return yuv;
    }

    GrayscaledImage getGrayscaled() const {
        GrayscaledImage ret(width * 2, height);
        unsigned char* dest = ret[0];
        const PixelTypes::YUYVPixel* src = (*this)[0];
        for(unsigned y = 0; y < width; y++) {
            for(unsigned x = 0; x < height; x++) {
                *dest++ = src->y0;
                *dest++ = src->y1;
                src++;
            }
        }
        return ret;
    }

};
