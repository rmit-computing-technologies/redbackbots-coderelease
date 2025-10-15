/**
 * Defines scan-line regions that segment a scan-line into multiple parts with an assigned color classification.
 *
 * @author Lukas Malte Monnerjahn
 * @author Felix
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
 */

#pragma once

#include <vector>

union ScanLineRange     // Not sure how to approach this, just left it for now  
{
  ScanLineRange() = default;
  ScanLineRange(const unsigned short from, const unsigned short to) : from(from), to(to) {}

  struct
  {
    unsigned short from; // upper/left; inclusive
    unsigned short to; // lower/right; exclusive
  };
  struct
  {
    unsigned short upper; // inclusive
    unsigned short lower; // exclusive
  };
  struct
  {
    unsigned short left; // inclusive
    unsigned short right; // exclusive
  };
};

struct ScanLineRegion
{
  enum Color // color classes for segmentation
  {
    unset, // no color decided yet
    black,
    white,
    field,
    none,  // any color that is not white or field
  };

  ScanLineRegion() = default;
  ScanLineRegion(unsigned short from, unsigned short to, Color c);

  ScanLineRange range{};
  Color color = Color::unset;

};

inline ScanLineRegion::ScanLineRegion(const unsigned short from, const unsigned short to, const Color c) :
  range(from, to), color(c)
{}

class ColorScanLineRegionsVertical {
public:

  class ScanLine {
  public:
    ScanLine() = default;
    explicit ScanLine(const unsigned short x);
    unsigned short x = 0;
    std::vector<ScanLineRegion> regions;
  };

  void draw() const;

  std::vector<ScanLine> scanLines;
  unsigned lowResStart = 0; /**< First index of low res scanLines. */
  unsigned lowResStep = 1;  /**< Steps between low res scanLines. */
};

inline ColorScanLineRegionsVertical::ScanLine::ScanLine(const unsigned short x) :
  x(x), regions()
{
  regions.reserve(32);
}

/**
 * A version of the ColorScanLineRegionsVertical that has been clipped at the FieldBoundary
 */
class ColorScanLineRegionsVerticalClipped: public ColorScanLineRegionsVertical
{
public:
  void draw() const;
};

/**
 * A version of the ColorScanLineRegionsVertical which is scaled (linear interpolated?)
 */
class CompressedColorScanLineRegionsVertical: public ColorScanLineRegionsVertical
{
public:
  void draw() const;
};

class ColorScanLineRegionsHorizontal
{
public:
  class ScanLine
  {
  public:
    ScanLine() = default;
    explicit ScanLine(const unsigned short y);
    unsigned short y = 0;
    std::vector<ScanLineRegion> regions;
  };

  void draw() const;

  std::vector<ScanLine> scanLines;
};

inline ColorScanLineRegionsHorizontal::ScanLine::ScanLine(const unsigned short y) :
  y(y), regions()
{
  regions.reserve(32);
}
