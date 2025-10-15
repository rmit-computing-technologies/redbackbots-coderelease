#pragma once

// #include "c++/9/bits/std_abs.h"
#include <cmath>

#define M2_PI 2.f*M_PI
/**
 * Converts angle from rad to degrees.
 * @param angle code in rad
 * @return angle coded in degrees
 */
template<typename V>
constexpr V toDegrees(V angle) { return angle * V(180.f / M_PI); }

/**
 * The Angle class stores the represented angle in radiant.
 */
class Angle
{
public:
  constexpr Angle() = default;
  constexpr Angle(float angle) : value(angle) {}

  operator float& () { return value; }
  constexpr operator const float& () const { return value; }

  constexpr Angle operator-() const { return Angle(-value); }
  Angle& operator+=(float angle) { value += angle; return *this; }
  Angle& operator-=(float angle) { value -= angle; return *this; }
  Angle& operator*=(float angle) { value *= angle; return *this; }
  Angle& operator/=(float angle) { value /= angle; return *this; }

  Angle& normalize() { value = normalize(value); return *this; }

  /**
   * reduce angle to [-pi..+pi[
   * @param data angle coded in rad
   * @return normalized angle coded in rad
   */
  template<typename V>
  static V normalize(V data);

  Angle diffAbs(Angle b) const { return std::abs(normalize(value - b)); }

  static constexpr Angle fromDegrees(float degrees) { return Angle((degrees / 180.f) * M_PI); }
  static constexpr Angle fromDegrees(int degrees) { return fromDegrees(static_cast<float>(degrees)); }

  constexpr float toDegrees() const { return (value / M_PI) * 180.f; }

private:
  float value = 0.f;
};

inline constexpr Angle operator "" _deg(unsigned long long int angle)
{
  return Angle::fromDegrees(static_cast<float>(angle));
}

inline constexpr Angle operator "" _deg(long double angle)
{
  return Angle::fromDegrees(static_cast<float>(angle));
}

inline constexpr Angle operator "" _rad(unsigned long long int angle)
{
  return Angle(static_cast<float>(angle));
}

inline constexpr Angle operator "" _rad(long double angle)
{
  return Angle(static_cast<float>(angle));
}

template<typename V>
V Angle::normalize(V data)
{
  if(data >= -V(M_PI) && data < V(M_PI))
    return data;
  else
  {
    data = data - static_cast<float>(static_cast<int>(data / V(M2_PI))) * V(M2_PI);
    return data >= V(M_PI) ? V(data - V(M2_PI)) : data < -V(M_PI) ? V(data + V(M2_PI)) : data;
  }
}

#ifndef isfinite
namespace std
{
  inline bool isfinite(Angle angle) noexcept
  {
    return isfinite(static_cast<float>(angle));
  }
}
#endif
