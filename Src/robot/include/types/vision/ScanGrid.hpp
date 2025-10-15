
/**
 * @file ScanGrid.hpp
 * 
 * A representation that describes the image grid that should be scanned.
 * 
 * @author Thomas Röfer
 * @author RedbackBots
 */


#pragma once

#include <vector>

class ScanGrid {
public:
    /**
     * Internal Line representation only for ScanGrid
     */
    struct ScanGridLine {
        ScanGridLine() = default;
        ScanGridLine(int x, int yMin, int yMax, unsigned lowResYMaxIndex, unsigned yMaxIndex);

        int x; /**< x coordinate of the scanLine. */
        int yMin; /**< Minimum y coordinate (inclusive). */
        int yMax; /**< Maximum y coordinate (exclusive). */
        unsigned lowResYMaxIndex; /**< Index of the lowest low resolution horizontal scanLine relevant for this scanLine. */
        unsigned yMaxIndex; /**< Index of the lowest y coordinate relevant for this scanLine. */
    };

    struct ScanGridHorizontalLine {
        ScanGridHorizontalLine() = default;
        ScanGridHorizontalLine(int y, int left, int right);

        int y; /**< y coordinates of the scanLine. */
        int left; /**< Left boundary of the scanLine (inclusive). */
        int right; /**< Right boundary of the scanLine (exclusive). */
    };

    void clear() {
        fullResY.clear();
        lowResHorizontalLines.clear();
        verticalLines.clear();
    }

    bool isValid() const {
        return !fullResY.empty() && !lowResHorizontalLines.empty() && !verticalLines.empty() && fieldLimit >= 0;
    }

    std::vector<int> fullResY; /**< All heights for a full resolution scan */
    std::vector<ScanGridHorizontalLine> lowResHorizontalLines; /**< Description of all horizontal scanLines. */
    std::vector<ScanGridLine> verticalLines; /**< Description of all vertical scanLines. */
    int fieldLimit = 0; /**< Upper bound for all scanLines (exclusive). */
    unsigned lowResStart = 0; /**< First index of low res grid. */
    unsigned lowResStep = 1; /**< Steps between low res grid lines. */
};

inline ScanGrid::ScanGridLine::ScanGridLine(int x, int yMin, int yMax, unsigned lowResYMaxIndex, unsigned yMaxIndex) :
  x(x), yMin(yMin), yMax(yMax), lowResYMaxIndex(lowResYMaxIndex), yMaxIndex(yMaxIndex)
{}

inline ScanGrid::ScanGridHorizontalLine::ScanGridHorizontalLine(int y, int left, int right) :
  y(y), left(left), right(right)
{}