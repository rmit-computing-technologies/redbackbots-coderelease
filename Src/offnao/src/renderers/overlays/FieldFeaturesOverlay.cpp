
/**
 * @file FieldFeaturesOverlay.cpp
 * 
 * Draw the field features
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/FieldFeaturesOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "types/FieldFeatureInfo.hpp"

FieldFeaturesOverlay::FieldFeaturesOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void FieldFeaturesOverlay::_drawOverlay(Blackboard* blackboard) {
    // get field boundary
    std::vector<FieldFeatureInfo> features = readFrom(vision, fieldFeatures);

    save();

    for(int i = 0; i < features.size(); ++i) {
        if(features[i].topCamera && whichCamera == CameraInfo::Camera::top || !features[i].topCamera && whichCamera == CameraInfo::Camera::bot){
            if(features[i].type == FieldFeatureInfo::Type::fLine) {
                setPen_(QColor("red"), defaultLineWidth);
                drawLine(v2i2q(features[i].p1), v2i2q(features[i].p2));
            }
            if(features[i].type == FieldFeatureInfo::Type::fCentreCircle) {
                setPen_(QColor("yellow"), 10);
                drawPoint(v2i2q(features[i].p1));
            }
            if(features[i].type == FieldFeatureInfo::Type::fPenaltySpot) {
                setPen_(QColor("green"), 10);
                drawPoint(v2i2q(features[i].p1));
            }
            if(features[i].type == FieldFeatureInfo::Type::fCorner || features[i].type == FieldFeatureInfo::Type::fTJunction || features[i].type == FieldFeatureInfo::Type::fXJunction) {
                setPen_(QColor("blue"), 10);
                drawPoint(v2i2q(features[i].p1));
            }
        }
    }

    restore();
}