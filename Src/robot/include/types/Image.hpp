/**
 * @author Alexis Tsogias
 * @author Nils Weller
 * @author Felix Thielke
 * @author Bernd Poppinga
 * @author RedbackBots
 */

#pragma once

#include "types/math/Eigen.hpp"
#include "types/PixelTypes.hpp"
#include <vector>

/**
 * Template class to represent an image of parameterizable pixel type.
 */
template<typename Pixel>
class Image {
public:
    using PixelType = Pixel;
    unsigned int width;
    unsigned int height;

private:
    std::vector<unsigned char> allocator;

protected:
    Pixel* image; /**< A pointer to the memory for the image */

public:
    Image() : width(0), height(0) {}
    Image(const unsigned int width, const unsigned int height, const unsigned int padding = 0) {
        setResolution(width, height, padding);
    }
    Image(const Image& other) {
        (*this) = other;
    }

    Image& operator=(const Image<Pixel>& other) {
        setResolution(other.width, other.height);
        memcpy(image, other[0], other.width * other.height * sizeof(Pixel));
        return *this;
    }

    Pixel* operator[](const size_t y) { 
        return image + y * width; 
    }
    const Pixel* operator[](const size_t y) const { 
        return image + y * width; 
    }
    Pixel& operator[](const Eigen::Matrix<short, 2, 1>& p) { 
        return image[p.y() * width + p.x()]; 
    }
    const Pixel& operator[](const Eigen::Matrix<short, 2, 1>& p) const { 
        return image[p.y() * width + p.x()]; 
    }
    Pixel& operator[](const Eigen::Vector2i& p) { 
        return image[p.y() * width + p.x()]; 
    }
    const Pixel& operator[](const Eigen::Vector2i& p) const { 
        return image[p.y() * width + p.x()]; 
    }

    /**
     * Get the pixel at x,y. No boundary check!
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The requested pixel.
     */
    PixelType& operator()(const size_t x, const size_t y) { 
        return *(image + (y * width + x)); 
    }

    /**
     * Get the const pixel at x,y. No boundary check!
     *
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The const requested pixel.
     */
    const PixelType& operator()(const size_t x, const size_t y) const { 
        return *(image + (y * width + x)); 
    }

    virtual void setResolution(const unsigned int width, const unsigned int height, const unsigned int padding = 0) {
        this->width = width;
        this->height = height;

        if(allocator.size() < width * height * sizeof(Pixel) + padding * 2) {
            allocator.resize(width * height * sizeof(Pixel) + 31 + padding * 2);
            image = reinterpret_cast<Pixel*>(reinterpret_cast<ptrdiff_t>(allocator.data() + 31 + padding) & (~ptrdiff_t(31)));
        }
    }

    // Friend class for serialisation/dumping
    friend class CameraImageDumper;
    friend class SerialiseImage;
};

using GrayscaledImage = Image<PixelTypes::GrayscaledPixel>;
using YUYVImage = Image<PixelTypes::YUYVPixel>;
using BGRAPixel = Image<PixelTypes::BGRAPixel>;

