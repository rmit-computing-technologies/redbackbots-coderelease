
/**
 * @file HorizonOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/FieldBoundaryOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "types/field/FieldBoundary.hpp"

FieldBoundaryOverlay::FieldBoundaryOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void FieldBoundaryOverlay::_drawOverlay(Blackboard* blackboard) {
    // get field boundary
    FieldBoundary fieldBoundary = readFrom(vision, fieldBoundary[whichCamera]);

    save();

    setPen_(QColor("brown"), defaultFieldBoundaryWidth);

    const std::vector<Vector2i>& boundaryInImage = fieldBoundary.boundaryInImage;
    for(int i = 1; i < boundaryInImage.size(); ++i) {
        drawLine(v2i2q(boundaryInImage[i-1]), v2i2q(boundaryInImage[i]));
    }

    restore();
}

