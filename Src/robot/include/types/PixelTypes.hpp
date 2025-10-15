#pragma once

#include "types/ColorModelConversions.hpp"
#include <vector>

namespace PixelTypes
{
  enum PixelType
  {
    RGB,              // useful for DebugImages?
    BGRA,             // the format that QImage uses
    YUYV,             // the format the NaoCamera supplies
    YUV,              // useful for DebugImages?
    Colored,          // format of the colored image in ECImage
    Grayscale,        // format of the gray-scaled image in ECImage
    Hue,              // hue channel of a YHS image
    Binary,           //
    Edge2,            //
    Edge2MonoAvg,     //
    Edge2MonoAbsAvg,  //
  };

  enum Color // color classes for segmentation
  {
    none,  // any color that is none of the others
    white,
    black,
    field,
  };

  struct RGBPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char padding;
      };
    };
  };

  struct BGRAPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char b;
        unsigned char g;
        unsigned char r;
        unsigned char a;
      };
    };

    static unsigned numPixel()
    {
      return 1;
    }

    std::vector<unsigned char> raw() const
    {
      return std::vector<unsigned char>{b, g, r};
    }
    std::vector<unsigned char> rgb() const
    {
      return std::vector<unsigned char>{r, g, b};
    }
    std::vector<unsigned char> grayscale() const
    {
      return std::vector<unsigned char>{static_cast<unsigned char>((r+g+b)/2)};
    }
  };

  struct YUYVPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        uint8_t y0;
        uint8_t u;
        uint8_t y1;
        uint8_t v;
      };

    };

    static unsigned numPixel()
    {
      return 2;
    }
    std::vector<uint8_t> raw() const
    {
      return std::vector<uint8_t>{y0, u, v, y1, u, v};
    }
    std::vector<uint8_t> rgb() const
    {
      std::vector<uint8_t> ret(6);
      ColorModelConversions::fromYUVToRGB(y0, u, v, ret[0], ret[1], ret[2]);
      ColorModelConversions::fromYUVToRGB(y1, u, v, ret[3], ret[4], ret[5]);
      return ret;
    }
    std::vector<uint8_t> grayscale() const
    {
      return std::vector<uint8_t>{y0,y1};
    }

    uint8_t& y(const size_t x)
    {
      return (reinterpret_cast<uint8_t*>(&color)[(x & 1) << 1]);
    }

    uint8_t y(const size_t x) const
    {
      return (reinterpret_cast<const uint8_t*>(&color)[(x & 1) << 1]);
    }

  };

  struct YUVPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char padding;
        unsigned char u;
        unsigned char y;
        unsigned char v;
      };
    };
  };

  struct HSIPixel
  {
    union
    {
      unsigned int color;
      struct
      {
        unsigned char h;
        unsigned char s;
        unsigned char i;
        unsigned char padding;
      };
    };
  };

  using ColoredPixel = Color;
  using GrayscaledPixel = unsigned char;



  class HuePixel
  {
  private:
    unsigned char value = 0;

  public:
    constexpr HuePixel() = default;
    constexpr HuePixel(unsigned char hue) : value(hue) {}

    operator unsigned char& () { return value; }
    constexpr operator const unsigned char& () const { return value; }

    constexpr HuePixel operator-() const { return -value; }
    HuePixel& operator+=(unsigned char hue) { value += hue; return *this; }
    HuePixel& operator-=(unsigned char hue) { value -= hue; return *this; }
    HuePixel& operator*=(unsigned char hue) { value *= hue; return *this; }
    HuePixel& operator/=(unsigned char hue) { value /= hue; return *this; }
  };

  using BinaryPixel = bool;

  struct Edge2Pixel { unsigned char filterX, filterY; Edge2Pixel() = default; Edge2Pixel(unsigned char e1, unsigned char e2) : filterX(e1), filterY(e2) {} };

  constexpr size_t pixelSize(const PixelType type)
  {
    return type == RGB ? sizeof(RGBPixel)
           : (type == BGRA ? sizeof(BGRAPixel)
              : (type == YUYV ? sizeof(YUYVPixel)
                 : (type == YUV ? sizeof(YUVPixel)
                    : (type == Grayscale ? sizeof(GrayscaledPixel)
                       : (type == Colored ? sizeof(ColoredPixel)
                          : (type == Hue ? sizeof(HuePixel)
                             : ((type == Edge2 || type == Edge2MonoAvg || type == Edge2MonoAbsAvg) ? sizeof(Edge2Pixel)
                                : (type == Binary ? sizeof(BinaryPixel)
                                   : 0))))))));
  }


}


