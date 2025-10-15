
/**
 * @file HorizonOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/FieldBoundaryPointsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "types/field/FieldBoundary.hpp"

FieldBoundaryPointsOverlay::FieldBoundaryPointsOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void FieldBoundaryPointsOverlay::_drawOverlay(Blackboard* blackboard) {
    // get field boundary
    FieldBoundary fieldBoundary = readFrom(vision, fieldBoundary[whichCamera]);

    save();

    setPen_(QColor("burlywood"), defaultFieldPointsWidth);

    const std::vector<Vector2i>& boundaryInImage = fieldBoundary.boundaryInImage;
    for(int i = 1; i < boundaryInImage.size(); ++i) {
        drawPoint(v2i2q(boundaryInImage[i]));
    }

    setPen_(QColor("green"), defaultFieldPointsWidth);
    const std::vector<Vector2i>& boundaryInImageLowerBound = fieldBoundary.boundaryInImageLowerBound;
    for(int i = 1; i < boundaryInImageLowerBound.size(); ++i) {

        drawPoint(v2i2q(boundaryInImageLowerBound[i]));
    }

    setPen_(QColor("green"), defaultFieldPointsWidth);
    const std::vector<Vector2i>& boundaryInImageUpperBound = fieldBoundary.boundaryInImageUpperBound;
    for(int i = 1; i < boundaryInImageUpperBound.size(); ++i) {
        drawPoint(v2i2q(boundaryInImageUpperBound[i]));
    }


    restore();
}

