/**
 * @file FieldFeatureDetector.cpp
 * 
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
*/

#include "perception/vision/detector/FieldFeatureDetector.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "types/math/Geometry.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/Logger.hpp"
#include "utils/debug/Assert.hpp"

#define MAX_LOCALISATION_FEATURES 50
#define MIN_FIELD_FEATURE_SEPARATION (300*300)
#define MIN_LINE_SIZE GOAL_BOX_WIDTH

#define DISTANCE_SQR(a, b) (SQUARE(a.x() - b.x()) + SQUARE(a.y() - b.y()))

FieldFeatureDetector::FieldFeatureDetector(Blackboard* blackboard):
Detector("FieldFeatureDetector")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "FieldFeatureDetector loaded" << std::endl;
}

FieldFeatureDetector::~FieldFeatureDetector() {

}

void FieldFeatureDetector::configure(Blackboard* blackboard) {

}

void FieldFeatureDetector::resetMiddleInfo(VisionInfoMiddle* info_middle) {
}

void FieldFeatureDetector::resetVisionOut(VisionInfoOut* info_out) {
    info_out->features.clear();
    numberOfFeaturesSent = 0;
}

void FieldFeatureDetector::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "-----FIELD FEATURES-----" << std::endl;

    // llog(INFO) << NDEBUG_LOGSYMB << "TOP" << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "---" << std::endl;
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // llog(INFO) << NDEBUG_LOGSYMB << "BOT" << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "---" << std::endl;
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);

    // llog(INFO) << NDEBUG_LOGSYMB << "------------------------" << std::endl;
}

void FieldFeatureDetector::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    
    const CameraImage* cameraImage = info_in->image[whichCamera];
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    int playerNum = info_in->playerNum;
    LineSpots& lineSpots = info_middle->lineSpots[whichCamera];
    CentreCircle& centreCircle = info_middle->centreCircle[whichCamera];
    Intersections& intersections = info_middle->intersections[whichCamera];
    PenaltyMarkInfo& penaltyMark = info_middle->penaltyMarkInfo[whichCamera];

    std::vector<FieldFeatureInfo>& features = info_out->features;
    
    // llog(DEBUG) << "Camera: " << cameraInfo.camera << std::endl;
    // llog(DEBUG) << "    - number of lines found: " << lineSpots.lines.size() << std::endl;
    // llog(DEBUG) << "    - is centre circle in camera:" << centreCircle.wasSeen << std::endl;
    // llog(DEBUG) << "    - number of intersections found: " << intersections.intersections.size() << std::endl;
    // llog(DEBUG) << "    - was penalty mark found: " << penaltyMark.wasSeen << std::endl;

    std::vector<size_t> sortedIndicies = processLines(info_in, cameraImage, cameraInfo, lineSpots, centreCircle, features);

    updateCentreCircle(info_in, cameraInfo, centreCircle, features);

    updateIntersections(info_in, cameraInfo, lineSpots, intersections, features);

    updatePenaltyMark(info_in, cameraInfo, penaltyMark, features);

    updateLines(cameraInfo, lineSpots, sortedIndicies, features, playerNum);

    // llog(DEBUG) << "    - number of features found: " << features.size() << std::endl;
    int numLines = 0;
    int numCentreCircles = 0;
    int numIntersections = 0;
    int numPenaltySpots = 0;

    for(const FieldFeatureInfo& ff : features) {
        if(ff.type == FieldFeatureInfo::Type::fLine) {
            ++numLines;
        }
        if(ff.type == FieldFeatureInfo::Type::fCentreCircle) {
            ++numCentreCircles;
        }
        if(ff.type == FieldFeatureInfo::Type::fTJunction || ff.type == FieldFeatureInfo::Type::fCorner || ff.type == FieldFeatureInfo::Type::fXJunction) {
            ++numIntersections;
        }
        if(ff.type == FieldFeatureInfo::Type::fPenaltySpot) {
            ++numPenaltySpots;
        }
    }

    // llog(INFO) << NDEBUG_LOGSYMB << "lines: " << numLines << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "centre circles: " << numCentreCircles << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "intersections: " << numIntersections << std::endl;

    lastTimestamp = cameraImage->timestamp;
}

std::vector<size_t> FieldFeatureDetector::processLines(const VisionInfoIn* info_in, const CameraImage* cameraImage, const CameraInfo& cameraInfo, LineSpots& lineSpots, CentreCircle& centreCircle, std::vector<FieldFeatureInfo>& features) {
    internalListOfLines.clear();
    spotLineUsage.clear();

    midLine = nullptr;

    for(LineSpots::Line& line : lineSpots.lines)
    {
        // TODO (MO)
        // if(theGameState.isPenaltyShootout())
        // {
        // if(std::abs(std::abs((theRobotPose * line.firstField - theRobotPose * line.lastField).angle()) - 90_deg) > 30_deg
        //     && (std::abs((theRobotPose * line.firstField).y()) < theFieldDimensions.yPosLeftPenaltyArea - 150
        //         || std::abs((theRobotPose * line.lastField).y()) < theFieldDimensions.yPosLeftPenaltyArea - 150))
        // {
        //     spotLineUsage.push_back(thrown);
        //     continue;
        // }

        // const Vector2f corner1(OPPONENT_FIELD_BOUNDARY, LEFT_FIELD_BORDER - 200);
        // const Vector2f corner2(OPPONENT_PENALTY_AREA / 2, -LEFT_FIELD_BORDER - 200);
        // if(!Geometry::isPointInsideRectangle2(corner1, corner2, theRobotPose * line.firstField)
        //     || !Geometry::isPointInsideRectangle2(corner1, corner2, theRobotPose * line.lastField))
        // {
        //     spotLineUsage.push_back(thrown);
        //     continue;
        // }

        if(line.belongsToCircle)
        {
            //FieldLines should not contain lines that are on the circle
            // llog(INFO) << NDEBUG_LOGSYMB << "line thrown - 1" << std::endl;
            spotLineUsage.push_back(thrown);
            continue;
        }

        //is line most likely in/of center circle
        if(centreCircle.wasSeen)
        {
            if(std::abs(Geometry::getDistanceToLine(line.line, centreCircle.pos) - (CENTER_CIRCLE_DIAMETER / 2)) < maxLineDeviationFromAssumedCenterCircle)
            {
                //(again) FieldLines should not contain lines that are on the circle
                // llog(INFO) << NDEBUG_LOGSYMB << "line thrown - 2" << std::endl;
                spotLineUsage.push_back(thrown);
                continue;
            }
        }
        
        //remove false-positives in balls
        const Vector2i centerInImage = (line.firstImg + line.lastImg) / 2;
        Vector2f relativePosition;
        Geometry::Circle ball;
        const float inImageRadius = -1.f;
        // TODO (MO)
        // const float inImageRadius = Transformation::imageToRobotHorizontalPlane(centerInImage.cast<float>(), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
        //                             && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
        if((line.firstImg - line.lastImg).squaredNorm() < static_cast<int>(SQUARE(inImageRadius * 2.5f) * std::abs(std::cos(line.line.direction.angle()))))
        {
            // llog(INFO) << NDEBUG_LOGSYMB << "line thrown - 3" << std::endl;
            spotLineUsage.push_back(thrown);
            continue;
        }
        

        Vector2f firstInField(line.firstField);
        Vector2f lastInField(line.lastField);
        Vector2i firstInImage(line.firstImg);
        Vector2i lastInImage(line.lastImg);
        
        spotLineUsage.emplace_back(stayed);
        
        ASSERT(std::isfinite(firstInField.x()));
        ASSERT(std::isfinite(firstInField.y()));
        ASSERT(std::isfinite(lastInField.x()));
        ASSERT(std::isfinite(lastInField.y()));
        
        internalListOfLines.emplace_back();
        LineSpots::Line& pLine = internalListOfLines.back();
        if(centreCircle.wasSeen)
        {
            //check if this is the midline
            Vector2f intersectionA;
            Vector2f intersectionB;
            const int numIntersections = Geometry::getIntersectionOfLineAndCircle(line.line, 
                                        Geometry::Circle(centreCircle.pos, (CENTER_CIRCLE_DIAMETER / 2)), intersectionA, intersectionB);
            if(numIntersections == 2)
            {
                //if the line intersects the circle twice
                //check if both intersections are between lineStart and lineEnd
                if(isPointInSegment(line, intersectionA) || isPointInSegment(line, intersectionB) ||
                    ((centreCircle.pos - line.firstField).squaredNorm() < SQUARE(CENTER_CIRCLE_DIAMETER / 2) &&
                    (centreCircle.pos - line.lastField).squaredNorm() < SQUARE(CENTER_CIRCLE_DIAMETER / 2) &&
                    (firstInField - lastInField).squaredNorm() > bigLineThreshold*bigLineThreshold / 2))
                {
                    midLine = &line;
                }
            }
        }
        // Fill elements of FieldLine representation:
        pLine.firstField = firstInField;
        pLine.lastField = lastInField;

        pLine.firstImg = firstInImage;
        pLine.lastImg = lastInImage;

        pLine.length = (firstInField - lastInField).norm();
    }

    // Sort final list of lines from long to short:
    std::vector<size_t> sortedLineIndizes;
    for(size_t i = 0; i < internalListOfLines.size(); i++) {
        sortedLineIndizes.emplace_back(i);
    }
    std::sort(sortedLineIndizes.begin(), sortedLineIndizes.end(), [&](const size_t a, const size_t b)
    {
        return internalListOfLines[a].length > internalListOfLines[b].length;
    });

    return sortedLineIndizes; 
}

void FieldFeatureDetector::updateLines(const CameraInfo& cameraInfo, LineSpots& lineSpots, std::vector<size_t> sortedLineIndizes, std::vector<FieldFeatureInfo>& features, int playerNum) {
    // Copy sorted list to representation:
    if(playerNum != 1){
        // llog(DEBUG) << "Don't use lines for vision" << std::endl;
        return; // Don't use lines unless we are goalie!
    }
    // llog(DEBUG) << "I am a goalie, so I will use lines" << std::endl;
    for(size_t i = 0; i < sortedLineIndizes.size(); i++) {
        const LineSpots::Line& sFieldLine = internalListOfLines[sortedLineIndizes[i]];
        Point p1 = sFieldLine.firstImg;
        Point p2 = sFieldLine.lastImg;

        PointF field1 = sFieldLine.firstField;
        PointF field2 = sFieldLine.lastField;

        if(sFieldLine.length < MIN_LINE_SIZE) {
            break;
        }

        // Get closest point to line (https://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd)
        int num =  (-p1.dot((p2  - p1)));
        int den = ((p2 - p1).dot((p2 - p1)));
        if (den == 0) den = 1;
        float s_x = p1.x() + (p2.x() - p1.x()) * 1.0f * num / den;
        float s_y = p1.y() + (p2.y() - p1.y()) * 1.0f * num / den;
        Point s(s_x, s_y);

        // Create RRCoord, using distance and heading to the closest point on line from robot
        float angle = NORMALISE(std::atan2(s.y(), s.x()));
        float shortestDistance = std::sqrt(s.y() * s.y() + s.x() * s.x());
        RRCoord c1(shortestDistance, angle);

        // Calculate distance and heading uncertainty for state estimation use
        Point midPoint = (p1 + p2)/2;
        float distanceToTheMidPoint = std::sqrt(midPoint.x() * midPoint.x() + midPoint.y() * midPoint.y());

        // equation of linear function to have:
        //  - standard deviation of 400mm at observation distance 1000mm, and
        //  - standard deviation of 1000mm at observation distance 4000mm
        float distUncertainty = 0.2 * distanceToTheMidPoint + 200;

        // equation of linear function to have:
        //  - standard deviation of 20deg at observation distance 1000mm, and
        //  - standard deviation of 30deg at observation distance 4000mm
        float headingUncertainty = 0.00005809 * distanceToTheMidPoint + 0.290973;

        c1.var(0,0) = distUncertainty * distUncertainty;
        c1.var(1,1) = headingUncertainty * headingUncertainty;
        c1.var(2,2) = 0;

        // Create Line Feature
        FieldFeatureInfo f1 = FieldFeatureInfo(c1, FieldFeatureInfo::fLine);

        // Check if this feature is too close to any existing features.
        bool tooClose = false;
        for(unsigned int featureID=0; featureID < features.size(); ++featureID)
        {
            if(f1.rr.distanceSquared(features[featureID].rr) < MIN_FIELD_FEATURE_SEPARATION)
            {
                tooClose = true;
                break;
            }
        }

        if(!tooClose){
            f1.p1 = p1;
            f1.p2 = p2;
            f1.field1 = field1;
            f1.field2 = field2;
            f1.topCamera = cameraInfo.camera == CameraInfo::Camera::top;
            if(!std::isnan(shortestDistance) && !std::isnan(angle) && !std::isnan(distUncertainty) && !std::isnan(headingUncertainty)) {
                // llog(INFO) << NDEBUG_LOGSYMB << "Added line to field features" << std::endl;
                ++numberOfFeaturesSent;
                features.emplace_back(f1);
            }
        }

        if(numberOfFeaturesSent >= MAX_LOCALISATION_FEATURES) {
            return;
        }
    }
}

void FieldFeatureDetector::updateCentreCircle(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, CentreCircle& centreCircle, std::vector<FieldFeatureInfo>& features) {
    if(centreCircle.wasSeen) {
        float distance = std::sqrt(
            SQUARE(centreCircle.pos.x()) +
            SQUARE(centreCircle.pos.y())
        );
        float heading = std::atan2(centreCircle.pos.y(), centreCircle.pos.x());
        RRCoord coord = RRCoord(distance, heading);

        FieldFeatureInfo cc = FieldFeatureInfo(coord, FieldFeatureInfo::fCentreCircle);

        if (midLine != nullptr){
            float angle = getRobotCentreAngle(cc) - getCentreLineAngle(midLine);
            if (angle < 0) {
                angle += M_PI;
            }
            cc.rr.orientation() = angle;
        }

        cc.p1 = info_in->kinematicPose.robotToImageXY(centreCircle.pos, cameraInfo);
        cc.topCamera = cameraInfo.camera == CameraInfo::Camera::top;

        ++numberOfFeaturesSent;
        features.emplace_back(cc);
    }
}

void FieldFeatureDetector::updateIntersections(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, LineSpots& lineSpots, Intersections& intersections, std::vector<FieldFeatureInfo>& features) {
    ASSERT(spotLineUsage.size() == lineSpots.lines.size());

    for(const Intersections::Intersection& i : intersections.intersections)
    {
        if(spotLineUsage[i.line1Index] == thrown || spotLineUsage[i.line2Index] == thrown) {
            continue;
        }

        FieldFeatureInfo intersectionFF;
        intersectionFF.p1 = i.img;
        intersectionFF.topCamera = cameraInfo.camera == CameraInfo::Camera::top;
        intersectionFF.rr = info_in->kinematicPose.RXYToRobotRelative(i.pos, cameraInfo);

        Vector2f intersectionRXY = i.pos;

        if (i.type == Intersections::Intersection::IntersectionType::L) {
            intersectionFF.type = FieldFeatureInfo::Type::fCorner;

            // The angle of the corner intersection.
            float angle = findCAngle(intersectionRXY, lineSpots.lines[i.line1Index], lineSpots.lines[i.line2Index]);
            intersectionFF.rr.orientation() = -angle;
        }
        else if (i.type == Intersections::Intersection::IntersectionType::T) {
            /* ----- Throw out T intersection if on center circle ----- */
            auto equalLines = [](const LineSpots::Line* midline, const LineSpots::Line l2)
            {
                return midline->firstField == l2.firstField && midline->lastField == l2.lastField;
            };

            [[maybe_unused]] const bool line1IsMid = midLine && equalLines(midLine, lineSpots.lines[i.line1Index]);
            const bool line2IsMid = midLine && equalLines(midLine, lineSpots.lines[i.line2Index]);

            ASSERT(!line1IsMid || !line2IsMid);
            if((i.type == Intersections::Intersection::T && line2IsMid) || lineSpots.lines[i.line1Index].belongsToCircle || lineSpots.lines[i.line2Index].belongsToCircle) {
                continue;
            }
            /* -------------------------------------------------------- */

            intersectionFF.type = FieldFeatureInfo::Type::fTJunction;

            // The angle of the T intersection.
            float angle = findTAngle(intersectionRXY, lineSpots.lines[i.line1Index]);
            intersectionFF.rr.orientation() = -angle;
        }
        else if (i.type == Intersections::Intersection::IntersectionType::X) {
            intersectionFF.type = FieldFeatureInfo::Type::fXJunction;
        }

        bool tooClose = false;

        // Check if this feature is too close to any existing features.
        for(unsigned int featureID=0; featureID < features.size(); ++featureID)
        {
            if(intersectionFF.rr.distanceSquared(features[featureID].rr) < MIN_FIELD_FEATURE_SEPARATION)
            {
                tooClose = true;
                break;
            }
        }

        if(!tooClose) {
            ++numberOfFeaturesSent;
            features.emplace_back(intersectionFF);

            // If too many features have been detected, exit.
            // if(numberOfFeaturesSent >= MAX_LOCALISATION_FEATURES){
            //     return;
            // }
        }
    }
}

void FieldFeatureDetector::updatePenaltyMark(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, PenaltyMarkInfo& penaltyMark, std::vector<FieldFeatureInfo>& features) {
    if(penaltyMark.wasSeen) {
        FieldFeatureInfo penaltyMarkFF;

        penaltyMarkFF.rr = info_in->kinematicPose.imageToRobotRelative(penaltyMark.positionInImage, cameraInfo);
        penaltyMarkFF.p1 = penaltyMark.positionInImage;
        penaltyMarkFF.type = FieldFeatureInfo::Type::fPenaltySpot;
        penaltyMarkFF.topCamera = cameraInfo.camera == CameraInfo::Camera::top;

        ++numberOfFeaturesSent;
        features.emplace_back(penaltyMarkFF);
    }
}

bool FieldFeatureDetector::isPointInSegment(const LineSpots::Line& line, const Vector2f& point) const
{
const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
return (line.firstField - point).squaredNorm() <= lineSquaredNorm && (line.lastField - point).squaredNorm() <= lineSquaredNorm;
}

float FieldFeatureDetector::getRobotCentreAngle(FieldFeatureInfo cc)
{
    Point directionRight = Point(0, 0);
    Point directionLeft = cc.rr.toCartesian();
    if (directionRight.y() > directionLeft.y()) {
        directionRight = cc.rr.toCartesian();
        directionLeft = Point(0, 0);
    }
    return std::atan2(
        directionRight.y() - directionLeft.y(),
        directionRight.x() - directionLeft.x()
    );
}

float FieldFeatureDetector::getCentreLineAngle(LineSpots::Line* line)
{
    Point centreRight = line->firstImg;
    Point centreLeft = line->lastImg;
    if (centreRight.y() > centreLeft.y()) {
    centreRight = line->lastImg;
    centreLeft = line->firstImg;
    }
    return std::atan2(
        centreRight.y() - centreLeft.y(),
        centreRight.x() - centreLeft.x()
    );
}

/*
Determines the angle of a corner in the form required by localisation.
*/
float FieldFeatureDetector::findCAngle(PointF& intersection, LineSpots::Line& l1, LineSpots::Line& l2) {
    float g1 = findGradient(l1, intersection);
    float g2 = findGradient(l2, intersection);
    float angle = (g1+g2)/2;
    float roughQuadrantLimit = (8*(M_PI/18));
    if ((g1 > 0) && (g2 < 0) && (g1 > roughQuadrantLimit) &&
                                                        (g2 < -roughQuadrantLimit))
    {
        g2 += 2*M_PI;
        angle = (g1+g2)/2;
    }
    if ((g2 > 0) && (g1 < 0) && (g2 > roughQuadrantLimit) &&
                                                        (g1 < -roughQuadrantLimit))
    {
        g1 += 2*M_PI;
        angle = (g1+g2)/2;
    }
    if (angle > M_PI) angle -= 2*M_PI;

    float theta = std::atan2(intersection.x(), intersection.y());

    if (angle > 0)
        angle = theta + M_PI - angle;
    else
        angle = theta - (M_PI + angle);

    if (angle > M_PI)
        angle = std::fmod(angle - M_PI, 2 * M_PI) + ((angle > 0) ? -M_PI : M_PI);

    return angle;
}

float FieldFeatureDetector::findTAngle(PointF& intersection, LineSpots::Line& l) {
float angle = findGradient(l, intersection);
float theta = std::atan2(intersection.x(), intersection.y());

if (angle > 0) {
    angle = theta + DEG2RAD(180) - angle;
} else {
    angle = theta - (DEG2RAD(180) + angle);
}

if (angle > M_PI) {
    angle = NORMALISE(angle);
    }
return angle;
}

float FieldFeatureDetector::findGradient(LineSpots::Line& l, PointF& intersection)
{
    // Work out which direction to find gradient in
    float distp1 = DISTANCE_SQR(intersection, l.firstField);
    float distp2 = DISTANCE_SQR(intersection, l.lastField);
    PointF far;
    PointF close;
    if (distp1 > distp2) {
        far = l.firstField;
        close = l.lastField;
    }
    else {
        far = l.lastField;
        close = l.firstField;
    }

    return (std::atan2(far.x() - close.x(), far.y() - close.y()));
}