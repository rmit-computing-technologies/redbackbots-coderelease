/**
 * @file LineAndCircleMiddleInfo.cpp
 *
 * Implements a module which detects lines and the center circle based on ColorScanLineRegions.
 *
 * @author Felix Thielke
 * @author Lukas Monnerjahn
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/LineAndCircleMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/SpatialUtilities.hpp" 
#include "types/PixelTypes.hpp"
#include "types/math/Deviation.hpp"
#include "types/math/Geometry.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/Logger.hpp"

LineAndCircleMiddleInfo::LineAndCircleMiddleInfo(Blackboard* blackboard):
  Detector("LineAndCircleMiddleInfo")
{
    configure(blackboard);

    spotsH.reserve(20);
    spotsV.reserve(20);
    candidates.reserve(50);
    circleCandidates.reserve(50);

    llog(INFO) << NDEBUG_LOGSYMB << "LineAndCircleMiddleInfo loaded" << std::endl;
}

LineAndCircleMiddleInfo::~LineAndCircleMiddleInfo() {

}

void LineAndCircleMiddleInfo::configure(Blackboard* blackboard) {

}

void LineAndCircleMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    // llog(DEBUG) << "TOP LINES: " << info_middle->lineSpots[CameraInfo::Camera::top].lines.size() << std::endl;
    // llog(DEBUG) << "BOT LINES: " << info_middle->lineSpots[CameraInfo::Camera::bot].lines.size() << std::endl;

    // llog(DEBUG) << "TOP H CANDIDATES: " << info_middle->candidates[CameraInfo::Camera::top].horizontalCandidates.size() << std::endl;
    // llog(DEBUG) << "TOP V CANDIDATES: " << info_middle->candidates[CameraInfo::Camera::top].verticalCandidates.size() << std::endl;

    // llog(DEBUG) << "BOT H CANDIDATES: " << info_middle->candidates[CameraInfo::Camera::bot].horizontalCandidates.size() << std::endl;
    // llog(DEBUG) << "BOT V CANDIDATES: " << info_middle->candidates[CameraInfo::Camera::bot].verticalCandidates.size() << std::endl;

    info_middle->lineSpots[CameraInfo::Camera::top].lines.clear();
    info_middle->lineSpots[CameraInfo::Camera::bot].lines.clear();

    info_middle->centreCircle[CameraInfo::Camera::top].reset();
    info_middle->centreCircle[CameraInfo::Camera::bot].reset();

    info_middle->candidates[CameraInfo::Camera::top].horizontalCandidates.clear();
    info_middle->candidates[CameraInfo::Camera::top].verticalCandidates.clear();
    info_middle->candidates[CameraInfo::Camera::bot].horizontalCandidates.clear();
    info_middle->candidates[CameraInfo::Camera::bot].verticalCandidates.clear();

    info_middle->candidatesBefore[CameraInfo::Camera::top].horizontalCandidates.clear();
    info_middle->candidatesBefore[CameraInfo::Camera::top].verticalCandidates.clear();
    info_middle->candidatesBefore[CameraInfo::Camera::bot].horizontalCandidates.clear();
    info_middle->candidatesBefore[CameraInfo::Camera::bot].verticalCandidates.clear();   

    info_middle->candidatesAfter[CameraInfo::Camera::top].horizontalCandidates.clear();
    info_middle->candidatesAfter[CameraInfo::Camera::top].verticalCandidates.clear();
    info_middle->candidatesAfter[CameraInfo::Camera::bot].horizontalCandidates.clear();
    info_middle->candidatesAfter[CameraInfo::Camera::bot].verticalCandidates.clear();

    info_middle->circleCandidates[CameraInfo::Camera::top].candidates.clear();
    info_middle->circleCandidates[CameraInfo::Camera::bot].candidates.clear();

    info_middle->IsWhiteSpots[CameraInfo::Camera::top].comparisons.clear();
    info_middle->IsWhiteSpots[CameraInfo::Camera::bot].comparisons.clear();
}

void LineAndCircleMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void LineAndCircleMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void LineAndCircleMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    LineSpots& lineSpots = info_middle->lineSpots[whichCamera];
    CentreCircle& centreCircle = info_middle->centreCircle[whichCamera];
    Candidates& debugCandidates = info_middle->candidates[whichCamera];

    Candidates& candidatesBefore = info_middle->candidatesBefore[whichCamera];
    Candidates& candidatesAfter = info_middle->candidatesAfter[whichCamera];
    CircleCandidates& debugCircleCandidates = info_middle->circleCandidates[whichCamera];
    IsWhiteSpots& isWhiteSpots = info_middle->IsWhiteSpots[whichCamera];

    // llog(DEBUG) << "WHICH CAMERA: " << whichCamera << std::endl;

    updateLineSpots(lineSpots, whichCamera, info_in, info_middle, info_out,
        debugCandidates, candidatesBefore, candidatesAfter, debugCircleCandidates,
        isWhiteSpots
    );

    updateCentreCircle(centreCircle, lineSpots, whichCamera, info_in, info_middle, info_out, isWhiteSpots);

    //llog(DEBUG) << "LineSpots size: " << lineSpots.lines.size() << std::endl;
    //llog(DEBUG) << "-------------------------------------" << std::endl;
}

void LineAndCircleMiddleInfo::updateLineSpots(
    LineSpots& lineSpots, CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle,
    VisionInfoOut* info_out, Candidates& debugCandidates, Candidates& candidatesBefore, Candidates& candidatesAfter,
    CircleCandidates& debugCircleCandidates, IsWhiteSpots& isWhiteSpots){
        const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
        FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];
        ECImage& ecImage = info_middle->ecImage[whichCamera];
        RelativeFieldColors& relativeFieldColors = info_middle->relativeFieldColors[whichCamera];
        RobotObstaclesImage robotsImage = info_middle->robotsImage[whichCamera];
        ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal = info_middle->colorScanLineRegionsHorizontal[whichCamera];
        ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped = info_middle->colorScanLineRegionsVerticalClipped[whichCamera];

        circleCandidates.clear();
        scanHorizontalScanLines(
            lineSpots, info_in, cameraInfo, fieldBoundary, colorScanLineRegionsHorizontal, ecImage, relativeFieldColors,
            debugCandidates, candidatesBefore, candidatesAfter, isWhiteSpots, robotsImage);
        scanVerticalScanLines(
            lineSpots, info_in, cameraInfo, colorScanLineRegionsVerticalClipped, ecImage,
            relativeFieldColors, debugCandidates, candidatesBefore, candidatesAfter, isWhiteSpots, robotsImage);
        extendLines(lineSpots, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots);

        debugCircleCandidates.candidates = circleCandidates;
        // llog(DEBUG) << "Reached end of updateLineSpots" << std::endl;
        // llog(DEBUG) << "Num of lines " << lineSpots.lines.size() << std::endl;
        // for (const LineSpots::Line& line : lineSpots.lines) {
        //     llog(DEBUG) << line.spotsInField.size() << std::endl;
        // }
}

void LineAndCircleMiddleInfo::updateCentreCircle(CentreCircle& centreCircle, LineSpots& lineSpots, 
 CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle,
 VisionInfoOut* info_out, IsWhiteSpots& isWhiteSpots) {
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];
    ECImage& ecImage = info_middle->ecImage[whichCamera];
    RelativeFieldColors& relativeFieldColors = info_middle->relativeFieldColors[whichCamera];

    // llog(DEBUG) << "Circle candidates: " << circleCandidates.size() << std::endl;
    if(!circleCandidates.empty()) {
        // find the best circle candidate to check extensively
        // the corrected circle position gets calculated only for a single candidate, to create an upper bound for the run time
        bool checkCandidate = false;
        size_t bestCandidateSpots = 0;
        CircleCandidate& bestCandidate = circleCandidates[0];

        // Find a valid center circle in the circle candidates
        for(CircleCandidate& candidate : circleCandidates) {
            if(candidate.fieldSpots.size() >= minSpotsOnCircle && candidate.circlePartInImage() > minCircleAngleBetweenSpots &&
            getAbsoluteDeviation(candidate.radius, (CENTER_CIRCLE_DIAMETER/2)) <= maxCircleRadiusDeviation &&
            candidate.calculateError() <= maxCircleFittingError) {
                if(candidate.fieldSpots.size() > bestCandidateSpots) {
                    checkCandidate = true;
                    bestCandidateSpots = candidate.fieldSpots.size();
                    bestCandidate = candidate;
                    // llog(DEBUG) << "best circle candidate updated" << std::endl;
                }
            }
        }

        // time intensive checks
        float lineError;
        if(checkCandidate && correctCircle(bestCandidate, info_in, cameraInfo, fieldBoundary, ecImage, relativeFieldColors, isWhiteSpots) &&
                isCircleWhite(bestCandidate.center, bestCandidate.radius, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
            //llog(DEBUG) << " -- center circle past first check " << std::endl;
            const bool accept = isCircleNotALine(bestCandidate, lineError);
            if(accept) {
                //llog(DEBUG) << " -- center circle past second check " << std::endl;
                centreCircle.pos = bestCandidate.center;
                centreCircle.image = info_in->kinematicPose.robotToImageXY(bestCandidate.center, cameraInfo);
                centreCircle.wasSeen = true;
                markLinesOnCircle(bestCandidate.center, lineSpots);
                // const Vector2f virtualPosForCovariance(bestCandidate.getAverageDistanceToFieldSpots(), 0.f);
                // centreCircle.cov = theMeasurementCovariance.computeForRelativePosition(virtualPosForCovariance);
                return;
            }
            // else{
            //     llog(DEBUG) << NDEBUG_LOGSYMB << "Circle rejected because it looks more like a line" << std::endl;
            // }
        }
    }
}

void LineAndCircleMiddleInfo::markLinesOnCircle(const Vector2f& center,  LineSpots& lineSpots) {
  for(const LineSpots::Line& line : lineSpots.lines) {
    for(const Vector2f& spot : line.spotsInField) {
      if(getAbsoluteDeviation((center - spot).norm(), (CENTER_CIRCLE_DIAMETER/2)) > maxCircleFittingError)
        goto lineNotOnCircle;
    }
    const_cast<LineSpots::Line&>(line).belongsToCircle = true;
  lineNotOnCircle :
    ;
  }
}

void LineAndCircleMiddleInfo::scanHorizontalScanLines(LineSpots& lineSpots, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal,
        ECImage& ecImage, RelativeFieldColors& relativeFieldColors, Candidates& debugCandidates,
        Candidates& candidatesBefore, Candidates& candidatesAfter, IsWhiteSpots& isWhiteSpots, RobotObstaclesImage& robotsImage) {
    spotsH.resize(colorScanLineRegionsHorizontal.scanLines.size());
    spotsH.clear();
    candidates.clear();
    debugCandidates.horizontalCandidates.clear();
    candidatesBefore.horizontalCandidates.clear();
    candidatesAfter.horizontalCandidates.clear();

    if (!fieldBoundary.isValid) {
        return;
    }

    unsigned int scanLineId = 0;
    for (const ColorScanLineRegionsHorizontal::ScanLine& scanLine : colorScanLineRegionsHorizontal.scanLines) {
        // llog(DEBUG) << "scanLine id: " << scanLineId << std::endl;
        spotsH[scanLineId].clear();
        spotsH[scanLineId].reserve(scanLine.regions.size() / 2);
        if (scanLine.regions.size() > 2) {
            //llog(DEBUG) << "scanLine.regions.size() > 2" << std::endl;
            for (auto region = scanLine.regions.cbegin() + 1; region != scanLine.regions.cend() - 1; ++region) {
                // llog(DEBUG) << " - region id: " << region - scanLine.regions.cbegin() << std::endl;
                // llog(DEBUG) << " - region->color: " << region->color << std::endl;
                if (region->color == ScanLineRegion::white) {
                    //llog(DEBUG) << " - region->color == ScanLineRegion::white" << std::endl;
                    auto before = region - 1;
                    auto after = region + 1;
                    for (int i = 0; i < maxSkipNumber 
                                    && before->range.right - before->range.left <= maxSkipWidth 
                                    && before != scanLine.regions.cbegin(); ++i, --before);
                    for (int i = 0; i < maxSkipNumber 
                                    && after->range.right - after->range.left <= maxSkipWidth 
                                    && after + 1 != scanLine.regions.cend(); ++i, ++after);
                    if (before->color == ScanLineRegion::field 
                        && after->color == ScanLineRegion::field 
                        && !isSpotInsideObstacle(robotsImage, region->range.left, region->range.right, scanLine.y, scanLine.y)) {
                        if (fieldBoundary.getBoundaryY((region->range.left + region->range.right) / 2) > static_cast<int>(scanLine.y)) {
                            // llog(DEBUG) << "   - goto hEndScan" << std::endl;
                            goto hEndScan;
                        }
                        spotsH[scanLineId].emplace_back(static_cast<float>((region->range.left + region->range.right)) / 2.f, scanLine.y);
                        Spot& thisSpot = spotsH[scanLineId].back();
                        candidatesBefore.horizontalCandidates.emplace_back(&thisSpot);
                        // Vector2f corrected(theImageCoordinateSystem.toCorrected(thisSpot.image));

                        // if(Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, thisSpot.field) &&
                        // Transformation::robotToImage(Vector2f(thisSpot.field + thisSpot.field.normalized(theFieldDimensions.fieldLinesWidth).rotateLeft()), theCameraMatrix, theCameraInfo, otherImage))
                        thisSpot.field = info_in->kinematicPose.imageToRobotXY(thisSpot.image, cameraInfo);
                        Vector2f otherImage = info_in->kinematicPose.robotToImageXY(Vector2f(thisSpot.field + thisSpot.field.normalized(FIELD_LINE_WIDTH).rotateLeft()), cameraInfo).cast<float>();
                        if(SpatialUtilities::possiblyOnFieldRRXY(thisSpot.field) && !(otherImage.x() < 0)) {
                            // llog(DEBUG) << "   - possiblyOnFieldRRXY" << std::endl;
                            float expectedWidth = (otherImage - thisSpot.image).norm();

                            // (BK) NOTE: POSSIBLE BUG HERE.
                            if(getAbsoluteDeviation(static_cast<int>(expectedWidth), region->range.right - region->range.left) <= maxLineWidthDeviationPx &&
                                (before->range.right - before->range.left >= static_cast<int>(expectedWidth * greenAroundLineRatio) || // as we do not currently use calibration we can just use greenAroundLineRatio
                                    (relaxedGreenCheckAtImageBorder && before->range.left == 0)) &&
                                (after->range.right - after->range.left >= static_cast<int>(expectedWidth * greenAroundLineRatio) || // as we do not currently use calibration we can just use greenAroundLineRatio
                                    (relaxedGreenCheckAtImageBorder && after->range.right == cameraInfo.width))) {
                                // llog(DEBUG) << "   - gotokeepHSpot" << std::endl;
                                goto keepHSpot;
                            }
                        }
                        spotsH[scanLineId].pop_back();
                        continue;

                    keepHSpot:
                        candidatesAfter.horizontalCandidates.emplace_back(&thisSpot);
                        //llog(DEBUG) << "   - keepHSpot" << std::endl;
                        if(scanLineId > 0) {
                            bool circleFitted = false, lineFitted = false;
                            for(CircleCandidate& candidate : circleCandidates) {
                                if(candidate.getDistance(thisSpot.field) <= maxCircleFittingError) {
                                    candidate.addSpot(thisSpot.field, thisSpot.image);
                                    circleFitted = true;
                                    //llog(DEBUG) << "    - circle fitted" << std::endl;
                                    break;
                                }
                            }
                            for(Candidate& candidate : candidates) {
                                if(candidate.spots.size() > 1 &&
                                candidate.getDistance(thisSpot.field) <= maxLineFittingError && 
                                isSegmentValid(thisSpot, *candidate.spots.back(), candidate, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                                    //llog(DEBUG) << "    - segment is valid" << std::endl;
                                    if(!circleFitted) {
                                        circleCandidates.emplace_back(candidate, thisSpot.field, thisSpot.image);
                                        if(circleCandidates.back().calculateError() > maxCircleFittingError) {
                                            llog(DEBUG) << "circleCandidates.back().calculateError() > maxCircleFittingError | "
                                            << circleCandidates.back().calculateError() << " > "
                                            << maxCircleFittingError << std::endl;
                                            circleCandidates.pop_back();
                                        }
                                        else {

                                            if(lineFitted) {
                                                //llog(DEBUG) << "     - goto hEndAdjacentSearch 1" << std::endl;
                                                goto hEndAdjacentSearch;
                                            }
                                            circleFitted = true;
                                        }
                                    }
                                    if(!lineFitted) {
                                        thisSpot.candidate = candidate.spots.front()->candidate;
                                        candidate.spots.emplace_back(&thisSpot);
                                        candidate.fitLine();
                                        if(circleFitted) {
                                            //llog(DEBUG) << "     - goto hEndAdjacentSearch 2" << std::endl;
                                            goto hEndAdjacentSearch;
                                        }
                                        lineFitted = true;
                                    }
                                }
                            }
                            if(lineFitted) {
                                //llog(DEBUG) << "    - goto hEndAdjacentSearch 3" << std::endl;
                                goto hEndAdjacentSearch;
                            }
                            for(const Spot& spot : spotsH[scanLineId - 1]) {
                                Candidate& candidate = candidates[spot.candidate];
                                if(candidate.spots.size() == 1 &&
                                getAbsoluteDeviation(spot.image.x(), thisSpot.image.x()) < getAbsoluteDeviation(spot.image.y(), thisSpot.image.y()) &&
                                isSegmentValid(thisSpot, spot, candidate, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                                    thisSpot.candidate = candidate.spots.front()->candidate;
                                    candidate.spots.emplace_back(&thisSpot);
                                    candidate.fitLine();
                                    //llog(DEBUG) << "    - goto hEndAdjacentSearch 4" << std::endl;
                                    goto hEndAdjacentSearch;
                                }
                            }
                        }
                        //llog(DEBUG) << "    - emplace back candidate" << std::endl;
                        thisSpot.candidate = static_cast<unsigned int>(candidates.size());
                        candidates.emplace_back(&thisSpot);
                    hEndAdjacentSearch:
                        ;
                    }
                }
            }
        }
        scanLineId++;
    }
    hEndScan:
    ;


    // if(!Transformation::imageToRobot(Vector2f(theCameraInfo.width / 2, theCameraInfo.height * 4 / 5), theCameraMatrix, theCameraInfo, lastNearPoint))
    Vector2f lastNearPoint = info_in->kinematicPose.imageToRobotXY(Vector2f(cameraInfo.width / 2, cameraInfo.height * 4 / 5), cameraInfo);
    if(!SpatialUtilities::possiblyOnFieldRRXY(lastNearPoint)) {
        // llog(DEBUG) << "OH no the possiblyOnFieldRRXY is something for " << CameraInfo::enumCameraToString(cameraInfo.camera) << std::endl;
        return;
    }
    const float maxNearDistance = std::max(1000.f, lastNearPoint.x());

    //llog(DEBUG) << "- number of candidates: " << candidates.size() << std::endl;
    for(const Candidate& candidate : candidates) {
        const float squaredLineLength = (candidate.spots.back()->field - candidate.spots.front()->field).squaredNorm();
        //llog(DEBUG) << "    - check if can emplace back line" << std::endl;
        // llog(DEBUG) << "        - candiate spots size: " << candidate.spots.size() << std::endl;
        if(candidate.spots.size() >= std::max<unsigned int>(2, minSpotsPerLine) && // as we do not currently use calibration we can just use minSpotsPerLine
         squaredLineLength >= minSquaredLineLength &&
         (candidate.spots.front()->field.x() <= maxNearDistance || squaredLineLength <= maxDistantHorizontalLength)) {
            const Spot* from, *to;
            const bool flipped = candidate.spots.front()->image.x() > candidate.spots.back()->image.x();
            if(flipped) {
                from = candidate.spots.front();
                to = candidate.spots.back();
            }
            else {
                from = candidate.spots.back();
                to = candidate.spots.front();
            }
            //llog(DEBUG) << "    - emplace back line horizontal" << std::endl;
            lineSpots.lines.emplace_back();
            LineSpots::Line& line = lineSpots.lines.back();
            line.firstField = from->field;
            line.lastField = to->field;
            line.line.base = line.firstField;
            line.line.direction = line.lastField - line.firstField;
            line.firstImg = static_cast<Vector2i>(from->image.cast<int>());
            line.lastImg = static_cast<Vector2i>(to->image.cast<int>());

            if(flipped) {
                for(auto it = candidate.spots.crbegin(); it != candidate.spots.crend(); it++) {
                    line.spotsInField.emplace_back((*it)->field);
                    line.spotsInImg.emplace_back(static_cast<Vector2i>((*it)->image.cast<int>()));
                }
            }
            else {
                for(const Spot* spot : candidate.spots) {
                    line.spotsInField.emplace_back(spot->field);
                    line.spotsInImg.emplace_back(static_cast<Vector2i>(spot->image.cast<int>()));
                }
            }
        }
    }

    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": Horizontal LineSpots size: " << lineSpots.lines.size() << std::endl;
    debugCandidates.horizontalCandidates = candidates;
    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": Before Horizontal Candidates size: " << candidatesBefore.horizontalCandidates.size() << std::endl;
    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": After Horizontal Candidates size: " << candidatesAfter.horizontalCandidates.size() << std::endl;

}

void LineAndCircleMiddleInfo::scanVerticalScanLines(LineSpots& lineSpots, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped, 
        ECImage& ecImage, RelativeFieldColors& relativeFieldColors, Candidates& debugCandidates,
        Candidates& candidatesBefore, Candidates& candidatesAfter, IsWhiteSpots& isWhiteSpots, RobotObstaclesImage& robotsImage) {
    spotsV.resize(colorScanLineRegionsVerticalClipped.scanLines.size());
    spotsV.clear();
    candidates.clear();
    debugCandidates.verticalCandidates.clear();
    candidatesBefore.verticalCandidates.clear();
    candidatesAfter.verticalCandidates.clear();

    unsigned int scanLineId = 0;
    unsigned int startIndex = highResolutionScan ? 0 : colorScanLineRegionsVerticalClipped.lowResStart;
    unsigned int stepSize = highResolutionScan ? 1 : colorScanLineRegionsVerticalClipped.lowResStep;
    for(unsigned scanLineIndex = startIndex; scanLineIndex < colorScanLineRegionsVerticalClipped.scanLines.size(); scanLineIndex += stepSize) {
        spotsV[scanLineId].clear();
        const std::vector<ScanLineRegion>& regions = colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].regions;
        spotsV[scanLineId].reserve(regions.size() / 2);
        if(regions.size() > 2) {
            for(auto region = regions.cbegin() + 1; region != regions.cend() - 1; ++region) {
                if(region->color == ScanLineRegion::white) {
                    auto before = region - 1;
                    auto after = region + 1;
                    for(int i = 0; i < maxSkipNumber && before->range.lower - before->range.upper <= maxSkipWidth && before != regions.cbegin(); ++i, --before);
                    for(int i = 0; i < maxSkipNumber && after->range.lower - after->range.upper <= maxSkipWidth && after + 1 != regions.cend(); ++i, ++after);
                    if(before->color == ScanLineRegion::field && after->color == ScanLineRegion::field &&
                    !isSpotInsideObstacle(robotsImage, colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, region->range.upper, region->range.lower)) {
                        spotsV[scanLineId].emplace_back(colorScanLineRegionsVerticalClipped.scanLines[scanLineIndex].x, static_cast<float>(region->range.upper + region->range.lower) / 2.f);
                        Spot& thisSpot = spotsV[scanLineId].back();
                        candidatesBefore.verticalCandidates.emplace_back(&thisSpot);

                        // Vector2f corrected = theImageCoordinateSystem.toCorrected(thisSpot.image);

                        // if(Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, thisSpot.field) &&
                        // Transformation::robotToImage(Vector2f(thisSpot.field + thisSpot.field.normalized(theFieldDimensions.fieldLinesWidth)), theCameraMatrix, theCameraInfo, otherImage))
                        thisSpot.field = info_in->kinematicPose.imageToRobotXY(thisSpot.image, cameraInfo);
                        Vector2f otherImage = info_in->kinematicPose.robotToImageXY(Vector2f(thisSpot.field + thisSpot.field.normalized(FIELD_LINE_WIDTH)), cameraInfo).cast<float>();
                        if(SpatialUtilities::possiblyOnFieldRRXY(thisSpot.field) && !(otherImage.x() < 0)) {
                            float expectedHeight = (thisSpot.image - otherImage).norm();
                            //llog(DEBUG) << "    - expectedHeight: " << expectedHeight * greenAroundLineRatio << std::endl;
                            //llog(DEBUG) << "    - before->range.lower - before->range.upper: " << before->range.lower - before->range.upper << std::endl;
                            //llog(DEBUG) << "    - after->range.lower - after->range.upper: " << after->range.lower - after->range.upper << std::endl;
                            if((before->range.lower - before->range.upper >= static_cast<int>(expectedHeight * greenAroundLineRatio) || // as we do not currently use calibration we can just use greenAroundLineRatio
                                    (relaxedGreenCheckAtImageBorder && before->range.lower == cameraInfo.height)) &&
                                (after->range.lower - after->range.upper >= static_cast<int>(expectedHeight * greenAroundLineRatio) || // as we do not currently use calibration we can just use greenAroundLineRatio
                                    (relaxedGreenCheckAtImageBorder && cameraInfo.camera == CameraInfo::Camera::bot && after->range.upper == 0))) {
                                //llog(DEBUG) << "    - goto keepVSpot" << std::endl;
                                goto keepVSpot;
                            }
                        }
                        spotsV[scanLineId].pop_back();
                        //llog(DEBUG) << "    - spotsV[scanLineId].pop_back()" << std::endl;
                        continue;

                    keepVSpot:
                        candidatesAfter.verticalCandidates.emplace_back(&thisSpot);
                    //llog(DEBUG) << "   - keepVSpot" << std::endl;
                        if(scanLineId > 0) {
                            bool circleFitted = false, lineFitted = false;
                            for(CircleCandidate& candidate : circleCandidates) {
                                if(candidate.getDistance(thisSpot.field) <= maxCircleFittingError) {
                                    candidate.addSpot(thisSpot.field, thisSpot.image);
                                    circleFitted = true;
                                    //llog(DEBUG) << "    - circle fitted: 1" << std::endl;
                                    break;
                                }
                            }
                            for(Candidate& candidate : candidates) {
                                if(candidate.spots.size() > 1 &&
                                candidate.getDistance(thisSpot.field) <= maxLineFittingError &&
                                isSegmentValid(thisSpot, *candidate.spots.back(), candidate, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                                    //llog(DEBUG) << "    - segment valid" << std::endl;
                                    if(!circleFitted) {
                                        circleCandidates.emplace_back(candidate, thisSpot.field, thisSpot.image);
                                        if(circleCandidates.back().calculateError() > maxCircleFittingError) {
                                            circleCandidates.pop_back();
                                        }
                                        else {
                                            if(lineFitted) {
                                                //llog(DEBUG) << "    - goto vEndAdjacentSearch: 1" << std::endl;
                                                goto vEndAdjacentSearch;
                                            }
                                            //llog(DEBUG) << "    - circle fitted: 2" << std::endl;
                                            circleFitted = true;
                                        }
                                    }
                                    if(!lineFitted) {
                                        thisSpot.candidate = candidate.spots.front()->candidate;
                                        candidate.spots.emplace_back(&thisSpot);
                                        candidate.fitLine();
                                        if(circleFitted) {
                                            //llog(DEBUG) << "    - goto vEndAdjacentSearch: 2" << std::endl;
                                            goto vEndAdjacentSearch;
                                        }
                                        //llog(DEBUG) << "    - line fitted" << std::endl;
                                        lineFitted = true;
                                    }
                                }
                            }
                            if(lineFitted) {
                                //llog(DEBUG) << "    - goto vEndAdjacentSearch: 3" << std::endl;
                                goto vEndAdjacentSearch;
                            }
                            for(const Spot& spot : spotsV[previousVerticalScanLine(thisSpot, static_cast<int>(scanLineId), colorScanLineRegionsVerticalClipped)]) {
                                Candidate& candidate = candidates[spot.candidate];
                                if(candidate.spots.size() == 1 &&
                                getAbsoluteDeviation(spot.image.x(), thisSpot.image.x()) > getAbsoluteDeviation(spot.image.y(), thisSpot.image.y()) &&
                                isSegmentValid(thisSpot, spot, candidate, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                                    thisSpot.candidate = candidate.spots.front()->candidate;
                                    candidate.spots.emplace_back(&thisSpot);
                                    candidate.fitLine();
                                    //llog(DEBUG) << "    - goto vEndAdjacentSearch: 4" << std::endl;
                                    goto vEndAdjacentSearch;
                                }
                            }
                        }
                        thisSpot.candidate = static_cast<unsigned int>(candidates.size());
                        candidates.emplace_back(&thisSpot);
                    vEndAdjacentSearch:
                        ;
                    }
                }
            }
        }
        scanLineId++;
    }
    // llog(DEBUG) << "- number of candidates: " << candidates.size() << std::endl;
    for(const Candidate& candidate : candidates) {
        //llog(DEBUG) << "    - check if can emplace back line" << std::endl;
        // llog(DEBUG) << "        - candiate spots size: " << candidate.spots.size() << std::endl;
        if(candidate.spots.size() >= std::max<unsigned int>(2, minSpotsPerLine)) { // as we do not currently use calibration we can just use minSpotsPerLine
            for(LineSpots::Line& line : lineSpots.lines) {
                if(candidate.getDistance(line.firstField) <= maxLineFittingError && candidate.getDistance(line.lastField) <= maxLineFittingError &&
                 isWhite(*candidate.spots.front(), Spot(line.lastImg.cast<float>(), line.lastField),
                    candidate.n0, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                    if(candidate.spots.front()->image.x() < static_cast<float>(line.firstImg.x())) {
                        line.firstField = candidate.spots.front()->field;
                        line.firstImg = static_cast<Vector2i>(candidate.spots.front()->image.cast<int>());
                    }
                    if(candidate.spots.back()->image.x() > static_cast<float>(line.lastImg.x())) {
                        line.lastField = candidate.spots.back()->field;
                        line.lastImg = static_cast<Vector2i>(candidate.spots.back()->image.cast<int>());
                    }
                    line.line.base = line.firstField;
                    line.line.direction = line.lastField - line.firstField;
                    for(const Spot* spot : candidate.spots) {
                        line.spotsInField.emplace_back(spot->field);
                        line.spotsInImg.emplace_back(static_cast<Vector2i>(spot->image.cast<int>()));
                    }
                    goto lineMerged;
                }
            }
            //llog(DEBUG) << "    - vertical line length: " << (candidate.spots.back()->field - candidate.spots.front()->field).squaredNorm() << std::endl;
            // llog(DEBUG) << "    candidates size: " << candidate.spots.size() << std::endl;
            if((candidate.spots.back()->field - candidate.spots.front()->field).squaredNorm() >= minSquaredLineLength) {
                //llog(DEBUG) << "    - emplace back line vertical" << std::endl;
                lineSpots.lines.emplace_back();
                LineSpots::Line& line = lineSpots.lines.back();
                line.firstField = candidate.spots.front()->field;
                line.lastField = candidate.spots.back()->field;
                line.line.base = line.firstField;
                line.line.direction = line.lastField - line.firstField;
                line.firstImg = static_cast<Vector2i>(candidate.spots.front()->image.cast<int>());
                line.lastImg = static_cast<Vector2i>(candidate.spots.back()->image.cast<int>());
                for(const Spot* spot : candidate.spots) {
                    lineSpots.lines.back().spotsInField.emplace_back(spot->field);
                    lineSpots.lines.back().spotsInImg.emplace_back(static_cast<Vector2i>(spot->image.cast<int>()));
                }
            }
        lineMerged:
            ;
        }
    }


    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": Vertical LineSpots size: " << lineSpots.lines.size() << std::endl;
    debugCandidates.verticalCandidates = candidates;
    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": Before Vertical Candidates size: " << candidatesBefore.verticalCandidates.size() << std::endl;
    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": After Vertical Candidates size: " << candidatesAfter.verticalCandidates.size() << std::endl;

    // llog(DEBUG) << CameraInfo::enumCameraToString(cameraInfo.camera) << ": Vertical Candidates size: " << candidates.size() << std::endl;
}

void LineAndCircleMiddleInfo::extendLines(LineSpots& lineSpots, const VisionInfoIn* info_in,
    const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors,
    IsWhiteSpots& isWhiteSpots) const {
    for(auto line = lineSpots.lines.begin(); line != lineSpots.lines.end();) {
        const Vector2f step(static_cast<Vector2f>((line->firstImg - line->lastImg).cast<float>().normalized() * 2.f));
        // Extend left
        Vector2f n0(-step.y(), step.x()); // rotates 90 degrees

        Vector2f pos(line->firstImg.cast<float>() + step);
        bool changed = false;
        for(; pos.x() >= 0 && pos.y() >= 0 && pos.x() < static_cast<float>(ecImage.grayscaled.width) &&
         pos.y() < static_cast<float>(ecImage.grayscaled.height); pos += step, changed = true) {
            // if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, pointOnField) || !isPointWhite(pointOnField, pos.cast<int>(), n0))
            Vector2f pointOnField = info_in->kinematicPose.imageToRobotXY(pos, cameraInfo);
            // isPointWhite type 1/2
            if(!SpatialUtilities::possiblyOnFieldRRXY(pointOnField) || !isPointWhite(pointOnField, pos.cast<int>(), n0, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                break;
            }
        }
        if(changed) {
            pos -= step;
            // if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
            Vector2f field = info_in->kinematicPose.imageToRobotXY(pos, cameraInfo);
            if(SpatialUtilities::possiblyOnFieldRRXY(field)) {
                line->firstImg = static_cast<Vector2i>(pos.cast<int>());
                line->firstField = field;
                line->spotsInImg.emplace_back(line->firstImg);
                line->spotsInField.emplace_back(line->firstField);
            }
        }
        // Extend right
        pos = line->lastImg.cast<float>() - step;
        changed = false;
        for(; pos.x() >= 0 && pos.y() >= 0 && pos.x() < static_cast<float>(ecImage.grayscaled.width) &&
         pos.y() < static_cast<float>(ecImage.grayscaled.height); pos -= step, changed = true) {
            // if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, pointOnField) || !isPointWhite(pointOnField, pos.cast<int>(), n0))
            Vector2f pointOnField = info_in->kinematicPose.imageToRobotXY(pos, cameraInfo);
            // isPointWhite type 1/2
            if(!SpatialUtilities::possiblyOnFieldRRXY(pointOnField) || !isPointWhite(pointOnField, pos.cast<int>(), n0, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                break;
            }
        }
        if(changed) {
            pos += step;
            // if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
            Vector2f field = info_in->kinematicPose.imageToRobotXY(pos, cameraInfo);
            if(SpatialUtilities::possiblyOnFieldRRXY(field)) {
                line->lastImg = static_cast<Vector2i>(pos.cast<int>());
                line->lastField = field;
                line->spotsInImg.emplace_back(line->lastImg);
                line->spotsInField.emplace_back(line->lastField);
            }
        }
        if(trimLines) { // as we do not currently use calibration we can just use trimLines
            trimLine(*line, info_in, cameraInfo, ecImage, relativeFieldColors);
        }

        // Recompute line
        line->line.base = line->firstField;
        line->line.direction = line->lastField - line->firstField;

        if(line->spotsInImg.size() < minSpotsPerLine || line->line.direction.squaredNorm() < minSquaredLineLength) { // as we do not currently use calibration we can just use minSpotsPerLine
            line = lineSpots.lines.erase(line);
        }
        else {
            ++line;
        }
    }
}

void LineAndCircleMiddleInfo::trimLine(LineSpots::Line& line, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors) const {
    Vector2i start = line.firstImg;
    Vector2i end = line.lastImg;
    if(start == end) {
        return;
    }

    for(int i = 0; i < 2; ++i) {
        int foundConsecutiveLineSpots = 0;

        Vector2f pos = i == 0 ? start.cast<float>() : end.cast<float>();
        Vector2f step = (i == 0 ? end - start : start - end).cast<float>().normalized() * 2.f;

        int left = line.firstImg.x() < line.lastImg.x() ? line.firstImg.x() : line.lastImg.x();
        int right = line.firstImg.x() < line.lastImg.x() ? line.lastImg.x() : line.firstImg.x();

        bool trimmed = false;
        for(; pos.y() >= 0 && pos.y() < static_cast<float>(ecImage.grayscaled.height) && pos.x() >= static_cast<float>(left) && pos.x() <= static_cast<float>(right); pos += step) {
            Vector2f dir = step / 2.f;
            dir.rotateLeft();
            Vector2f lower = pos - dir, upper = pos + dir;
            unsigned char luminanceReference = ecImage.grayscaled[static_cast<Vector2i>(pos.cast<int>())];
            unsigned char saturationReference = ecImage.saturated[static_cast<Vector2i>(pos.cast<int>())];
            int loopCount = 0;
            for(Vector2i p(lower.cast<int>());
             p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(ecImage.grayscaled.width) && p.y() < static_cast<int>(ecImage.grayscaled.height);
             lower -= dir, p = static_cast<Vector2i>(lower.cast<int>()), ++loopCount) {
                if(loopCount > maxWidthImage || relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[p], ecImage.saturated[p], luminanceReference, saturationReference)) {
                    break;
                }
            }
            lower += dir;
            for(Vector2i p(upper.cast<int>());
             p.x() >= 0 && p.y() >= 0 && p.x() < static_cast<int>(ecImage.grayscaled.width) && p.y() < static_cast<int>(ecImage.grayscaled.height);
             upper += dir, p = static_cast<Vector2i>(upper.cast<int>()), ++loopCount) {
                if(loopCount > maxWidthImage || relativeFieldColors.isFieldNearWhite(ecImage.grayscaled[p], ecImage.saturated[p],luminanceReference, saturationReference)) {
                    break;
                }
            }
            upper -= dir;

            if((upper - lower).squaredNorm() > maxWidthImageSquared) {
                // if(!Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(lower), theCameraMatrix, theCameraInfo, lowerField) ||
                // !Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(upper), theCameraMatrix, theCameraInfo, upperField) ||
                // (upperField - lowerField).norm() > theFieldDimensions.fieldLinesWidth * mFactor)
                Vector2f lowerField = info_in->kinematicPose.imageToRobotXY(lower, cameraInfo);
                Vector2f upperField = info_in->kinematicPose.imageToRobotXY(upper, cameraInfo);
                if(!SpatialUtilities::possiblyOnFieldRRXY(lowerField) || !SpatialUtilities::possiblyOnFieldRRXY(upperField) ||
                (upperField - lowerField).norm() > FIELD_LINE_WIDTH * mFactor) {
                    foundConsecutiveLineSpots = 0;
                    trimmed = true;
                    continue;
                }
                else {
                    ++foundConsecutiveLineSpots;
                }
            }
            else {
                ++foundConsecutiveLineSpots;
            }

            if(foundConsecutiveLineSpots == minConsecutiveSpots) {
                if(trimmed && pos.cast<int>() != start && pos.cast<int>() != end) {
                    // if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
                    Vector2f field = info_in->kinematicPose.imageToRobotXY(pos, cameraInfo);
                    if(SpatialUtilities::possiblyOnFieldRRXY(field)) {
                        line.spotsInImg.emplace_back(pos.cast<int>());
                        line.spotsInField.emplace_back(field);
                        pos -= (minConsecutiveSpots - 1) * step;
                        //if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(pos), theCameraMatrix, theCameraInfo, field))
                        field = info_in->kinematicPose.imageToRobotXY(pos, cameraInfo);
                        if(SpatialUtilities::possiblyOnFieldRRXY(field)) {
                            if(i == 0) {
                                line.firstImg = pos.cast<int>();
                                line.firstField = field;
                            }
                            else {
                                line.lastImg = pos.cast<int>();
                                line.lastField = field;
                            }
                            line.spotsInImg.emplace_back(pos.cast<int>());
                            line.spotsInField.emplace_back(field);
                        }
                    }
                }
                break;
            }
        }
    }

    if(start != line.firstImg || end != line.lastImg) {
        std::vector<Vector2i> spotsInImgTrimmed;
        spotsInImgTrimmed.reserve(line.spotsInImg.size());
        std::vector<Vector2f> spotsInFieldTrimmed;
        spotsInFieldTrimmed.reserve(line.spotsInField.size());
        for(unsigned int i = 0; i < line.spotsInImg.size(); ++i) {
            Vector2i spotInImage = line.spotsInImg.at(i);
            if(spotInImage.x() >= line.firstImg.x() && spotInImage.x() <= line.lastImg.x()) {
                spotsInImgTrimmed.emplace_back(spotInImage);
                spotsInFieldTrimmed.emplace_back(line.spotsInField.at(i));
            }
        }
        line.spotsInImg = spotsInImgTrimmed;
        line.spotsInField = spotsInFieldTrimmed;
    }
}

bool LineAndCircleMiddleInfo::isSegmentValid(const Spot& a, const Spot& b, const Candidate& candidate, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors,
        IsWhiteSpots& isWhiteSpots) {
    Vector2f n0 = (b.field - a.field);
    n0.rotateLeft();
    n0.normalize();
    if(candidate.spots.size() > 2) {
        float angleDiff = n0.angleTo(candidate.n0);
        angleDiff = angleDiff <= M_PI_2 ? angleDiff : M_PI - angleDiff;
        if(angleDiff > maxNormalAngleDiff) {
            //llog(DEBUG) << "   - segment not valid 1" << std::endl;
            return false;
        }
        n0 = candidate.n0;
    }
    return isWhite(a, b, n0, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots);
}

bool LineAndCircleMiddleInfo::isWhite(const Spot& a, const Spot& b, const Vector2f& n0Field, const VisionInfoIn* info_in,
        const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors,
        IsWhiteSpots& isWhiteSpots) {
    const Geometry::PixeledLine line(a.image.cast<int>(), b.image.cast<int>(), std::min(static_cast<int>(whiteCheckStepSize),
                                    static_cast<int>(std::ceil(static_cast<float>(std::max(getAbsoluteDeviation(a.image.x(), b.image.x()),
                                    getAbsoluteDeviation(a.image.y(), b.image.y()))) * minWhiteRatio))));

    const auto maxNonWhitePixels = static_cast<unsigned int>(static_cast<float>(line.size()) * (1 - minWhiteRatio));
    if(maxNonWhitePixels == line.size()) {
        //llog(DEBUG) << "   - white: true 1" << std::endl;
        return true;
    }

    if(perspectivelyCorrectWhiteCheck) {
        Vector2f pointOnField = a.field;
        unsigned int nonWhiteCount = 0;

        for(const Vector2i& p : line) {
            // if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(p), theCameraMatrix, theCameraInfo, pointOnField))
            pointOnField = info_in->kinematicPose.imageToRobotXY(p.cast<float>(), cameraInfo);
            if(SpatialUtilities::possiblyOnFieldRRXY(pointOnField)) {
                // isPointWhite type 1/2
                if(!isPointWhite(pointOnField, p, n0Field, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                    ++nonWhiteCount;
                    if(nonWhiteCount > maxNonWhitePixels) {
                        //llog(DEBUG) << "   - white: false 1" << std::endl;
                        return false;
                    }
                }
            }
        }
        return true;
        //llog(DEBUG) << "   - white: true 2" << std::endl;
    }
    else {
        Vector2f n0Image = b.image - a.image;
        n0Image.rotateLeft();
        n0Image.normalize();
        Vector2f pointOnField;
        float s = 1.f;
        unsigned int nonWhiteCount = 0;
        for(const Vector2i& p : line) {
            // if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(p), theCameraMatrix, theCameraInfo, pointOnField))
            pointOnField = info_in->kinematicPose.imageToRobotXY(p.cast<float>(), cameraInfo);
            if(SpatialUtilities::possiblyOnFieldRRXY(pointOnField)) {
                s = calcWhiteCheckDistanceInImage(pointOnField, cameraInfo);
            }
            else {
                //llog(DEBUG) << "   - white: false 2" << std::endl;
                return false;
            }
            // isPointWhite type 3/4
            if(!isPointWhite(p.cast<float>(), s * n0Image, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                ++nonWhiteCount;
                if(nonWhiteCount > maxNonWhitePixels) {
                    //llog(DEBUG) << "   - white: false 3" << std::endl;
                    return false;
                }
            }
        }
        //llog(DEBUG) << "   - white: true 3" << std::endl;
        return true;
    }
}

bool LineAndCircleMiddleInfo::correctCircle(CircleCandidate& circle, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage,
        RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const {
    // if(!Transformation::robotToImage(circle.center, theCameraMatrix, theCameraInfo, centerInImage))
    Vector2f centerInImage = info_in->kinematicPose.robotToImageXY(circle.center, cameraInfo).cast<float>();
    if(centerInImage.x() < 0) {
        return false;
    }

    // centerInImage = theImageCoordinateSystem.fromCorrected(centerInImage);

    circle.fieldSpots.clear();

    for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
    {
        Vector2f pointOnField(circle.center.x() + std::cos(a) * circle.radius, circle.center.y() + std::sin(a) * circle.radius);
        // if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
        Vector2f pointInImage = info_in->kinematicPose.robotToImageXY(pointOnField, cameraInfo).cast<float>();
        if(!(pointInImage.x() < 0)) {
            auto imageWidth = static_cast<float>(cameraInfo.width);
            auto imageHeight = static_cast<float>(cameraInfo.height);
            // pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
            if(pointInImage.x() >= 0 && pointInImage.x() < imageWidth && pointInImage.y() >= 0 && pointInImage.y() < imageHeight) {
                if(static_cast<float>(fieldBoundary.getBoundaryY(static_cast<int>(pointInImage.x()))) > pointInImage.y()) {
                    return false;
                }

                Vector2f dir(pointInImage - centerInImage);
                float s = calcWhiteCheckDistance(pointOnField) / circle.radius * dir.norm();
                dir.normalize();
                Vector2f n = s * dir;

                Vector2f outer(pointInImage);
                short maxLuminance = 0;
                int maxLoops = static_cast<int>(s) + 1;
                // isPointWhite type 3/4
                for(int loops = 0; loops <= maxLoops && !isPointWhite(outer, n, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots); ++loops) {
                    outer += dir;
                    if(loops == maxLoops || !(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight)) {
                        outer = pointInImage;
                        // isPointWhite type 3/4
                        for(int inwardLoops = 0; inwardLoops <= maxLoops && !isPointWhite(outer, n, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots); ++inwardLoops) {
                            outer -= dir;
                            if(inwardLoops == maxLoops || !(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight)) {
                                goto skipPoint;
                            }
                        }
                        break;
                    }
                }
                pointInImage = outer;
                do {
                    maxLuminance = std::max(maxLuminance, static_cast<short>(ecImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())]));
                    outer += dir;
                }
                // isPointWhite type 3/4
                while(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight &&
                 isPointWhite(outer, n, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots) && ecImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())] + 
                 relativeFieldColors.rfcParameters.minWhiteToFieldLuminanceDifference >= maxLuminance);
                outer -= dir;
                Vector2f inner(pointInImage);
                n = -n;
                // isPointWhite type 3/4
                while(!isPointWhite(inner, n, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                    inner -= dir;
                    if(!(inner.x() >= 0 && inner.x() < imageWidth && inner.y() >= 0 && inner.y() < imageHeight)) {
                        goto skipPoint;
                    }
                }
                do {
                    maxLuminance = std::max(maxLuminance, static_cast<short>(ecImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())]));
                    inner -= dir;
                }
                // isPointWhite type 3/4
                while(inner.x() >= 0 && inner.x() < imageWidth && inner.y() >= 0 && inner.y() < imageHeight &&
                 isPointWhite(inner, n, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots) && ecImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())] +
                 relativeFieldColors.rfcParameters.minWhiteToFieldLuminanceDifference >= maxLuminance);
                inner += dir;

                // outer = theImageCoordinateSystem.toCorrected(outer);
                // inner = theImageCoordinateSystem.toCorrected(inner);

                // if(Transformation::imageToRobot(outer, theCameraMatrix, theCameraInfo, outerField) &&
                // Transformation::imageToRobot(inner, theCameraMatrix, theCameraInfo, innerField) &&
                // (outerField - innerField).squaredNorm() <= circleCorrectionMaxLineWidthSquared)
                Vector2f outerField = info_in->kinematicPose.imageToRobotXY(outer, cameraInfo);
                Vector2f innerField = info_in->kinematicPose.imageToRobotXY(inner, cameraInfo);
                if(SpatialUtilities::possiblyOnFieldRRXY(outerField) && SpatialUtilities::possiblyOnFieldRRXY(innerField) && 
                (outerField - innerField).squaredNorm() <= circleCorrectionMaxLineWidthSquared) {
                    circle.fieldSpots.emplace_back((outerField + innerField) / 2);
                }
            }
        }

    skipPoint:
        ;
    }

    if(circle.fieldSpots.size() < minSpotsOnCircle) {
        return false;
    }

    LeastSquares::fitCircle(circle.fieldSpots, circle.center, circle.radius);

    for(size_t i = 0; i < circle.fieldSpots.size(); ++i) {
        Vector2f& spot = circle.fieldSpots[i];
        if(circle.getDistance(spot) > maxCircleFittingError) {
            do {
                spot = circle.fieldSpots.back();
                circle.fieldSpots.pop_back();
                if(circle.fieldSpots.size() < minSpotsOnCircle) {
                    return false;
                }
            }
            while(i < circle.fieldSpots.size() && circle.getDistance(spot) > maxCircleFittingError);
        }
    }

    circle.fitter = LeastSquares::CircleFitter();
    circle.fitter.add(circle.fieldSpots);

    return circle.fitter.fit(circle.center, circle.radius) &&
     getAbsoluteDeviation(circle.radius, (CENTER_CIRCLE_DIAMETER/2)) <= maxCircleRadiusDeviation &&
     circle.calculateError() <= maxCircleFittingError;
}

bool LineAndCircleMiddleInfo::isCircleWhite(const Vector2f& center, const float radius, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage,
        RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const {
    unsigned int whiteCount = 0, count = 0;
    for(Angle a = 0_deg; a < 360_deg; a += 5_deg) {
        Vector2f pointOnField(center.x() + std::cos(a) * radius, center.y() + std::sin(a) * radius);
        // if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
        Vector2f pointInImage = info_in->kinematicPose.robotToImageXY(pointOnField, cameraInfo).cast<float>();
        // pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
        if(pointInImage.x() >= 0 && pointInImage.x() < static_cast<float>(cameraInfo.width) &&
                pointInImage.y() >= 0 && pointInImage.y() < static_cast<float>(cameraInfo.height)) {
            count++;
            Vector2f n0(std::cos(a), std::sin(a)); // normal vector pointing outward
            // isPointWhite type 1/2
            if(isPointWhite(pointOnField, pointInImage.cast<int>(), n0, info_in, cameraInfo, ecImage, relativeFieldColors, isWhiteSpots)) {
                whiteCount++;
            }
        }
    }

    return count > 0 && static_cast<float>(whiteCount) / static_cast<float>(count) >= minCircleWhiteRatio;
}

bool LineAndCircleMiddleInfo::isCircleNotALine(const CircleCandidate& candidate, float& lineError) const {
    ASSERT(candidate.fieldSpots.size() >= 2);
    LeastSquares::LineFitter fitter;
    for(const Vector2f& spot : candidate.fieldSpots) {
        fitter.add(spot);
    }

    Vector2f n0;
    float d;
    fitter.fit(n0, d);

    lineError = 0.f;
    for(const Vector2f& spot : candidate.fieldSpots) {
        lineError += std::abs(n0.dot(spot) - d);
    }
    lineError /= candidate.fieldSpots.size();

    // llog(DEBUG) << "   - candidate.calculateError(): " << candidate.calculateError() << " VS lineError: " << lineError << std::endl;

    return candidate.calculateError() < lineError;
}

/**
 * Checks if the point in image is white. - Type 1 and 2
 * @param pointOnField The point on the field.
 * @param pointInImage The point in image coordinates.
 * @param n0 The normal vector pointing outward from the field.
 * @param info_in The vision information input.
 * @param cameraInfo The camera information.
 * @param ecImage The EC image.
 * @param relativeFieldColors The relative field colors.
 * @param isWhiteSpots The white spots information.
 * @return True if the point is white, false otherwise.
 */
bool LineAndCircleMiddleInfo::isPointWhite(const Vector2f& pointOnField, const Vector2i& pointInImage, const Vector2f& n0,
    const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const {
    Vector2f nw = calcWhiteCheckDistance(pointOnField) * n0;
    Vector2i referencePointInImage;
    Vector2f outerPointOnField = pointOnField + nw;
    Vector2f innerPointOnField = pointOnField - nw;
    unsigned short luminanceReference = 0, saturationReference = 0;
    bool isOuterPointInImage = false;

    //if(Transformation::robotToImage(outerPointOnField, theCameraMatrix, theCameraInfo, referencePointInImage))
    referencePointInImage = info_in->kinematicPose.robotToImageXY(outerPointOnField, cameraInfo);
    // referencePointInImage = theImageCoordinateSystem.fromCorrected(referencePointInImage);
    if(referencePointInImage.x() >= 0 && referencePointInImage.x() < cameraInfo.width &&
            referencePointInImage.y() >= 0 && referencePointInImage.y() < cameraInfo.height) {
        luminanceReference = ecImage.grayscaled[referencePointInImage];
        saturationReference = ecImage.saturated[referencePointInImage];
        isOuterPointInImage = true;

        Comparison comparison;
        comparison.pointInImage = Spot(pointInImage);
        comparison.referencePoint = Spot(referencePointInImage);
        comparison.type = 1;
        // if (comparison.hasZero()){
        //     llog(INFO) << "hasZero - 2 " << std::endl;
        //     llog(DEBUG) << "pointInImage: " << pointInImage.x() << "," << pointInImage.y() << " | referencePointInImage: " << referencePointInImage.x() << "," << referencePointInImage.y() << std::endl;
        // }
    }

    // if(Transformation::robotToImage(innerPointOnField, theCameraMatrix, theCameraInfo, referencePointInImage))
    referencePointInImage = info_in->kinematicPose.robotToImageXY(innerPointOnField, cameraInfo);
    // referencePointInImage = theImageCoordinateSystem.fromCorrected(referencePointInImage);
    if(referencePointInImage.x() >= 0 && referencePointInImage.x() < cameraInfo.width &&
            referencePointInImage.y() >= 0 && referencePointInImage.y() < cameraInfo.height) {
        if(isOuterPointInImage) {
            luminanceReference = (luminanceReference + ecImage.grayscaled[referencePointInImage] + 1) / 2;
            saturationReference = (saturationReference + ecImage.saturated[referencePointInImage]) / 2;
        }
        else {
            luminanceReference = ecImage.grayscaled[referencePointInImage];
            saturationReference = ecImage.saturated[referencePointInImage];
        }

        Comparison comparison;
        comparison.pointInImage = Spot(pointInImage);
        comparison.referencePoint = Spot(referencePointInImage);
        comparison.type = 2;
        // if (comparison.hasZero()){
        //     llog(INFO) << "hasZero - 3 " << std::endl;
        //     llog(DEBUG) << "pointInImage: " << pointInImage.x() << "," << pointInImage.y() << " | referencePointInImage: " << referencePointInImage.x() << "," << referencePointInImage.y() << std::endl;
        // }
        isWhiteSpots.comparisons.push_back(comparison);
    }
    return relativeFieldColors.isWhiteNearField(ecImage.grayscaled[pointInImage],ecImage.saturated[pointInImage],
            static_cast<unsigned char>(luminanceReference), static_cast<unsigned char>(saturationReference));
}
/**
 * Checks if the point in image is white. - Type 3 and 4
 * @param pointInImage The point in image coordinates.
 * @param n The normal vector pointing outward from the field.
 * @param cameraInfo The camera information.
 * @param ecImage The EC image.
 * @param relativeFieldColors The relative field colors.
 * @param isWhiteSpots The white spots information.
 * @return True if the point is white, false otherwise.
 */
bool LineAndCircleMiddleInfo::isPointWhite(const Vector2f& pointInImage, const Vector2f& n, const CameraInfo& cameraInfo, ECImage& ecImage,
        RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const {
    bool isOuterPointInImage = false;
    unsigned short luminanceReference = 0, saturationReference = 0;
    Vector2i outerReference = (pointInImage + n).cast<int>();
    if(outerReference.x() >= 0 && outerReference.x() < cameraInfo.width &&
        outerReference.y() >= 0 && outerReference.y() < cameraInfo.height) 
    {
        luminanceReference = ecImage.grayscaled[outerReference];
        saturationReference = ecImage.saturated[outerReference];
        isOuterPointInImage = true;
        Comparison comparison;
        comparison.pointInImage = Spot(pointInImage);
        comparison.referencePoint = Spot(outerReference);
        comparison.type = 3;
        // if (comparison.hasZero()){
        //     llog(INFO) << "hasZero - 4 " << std::endl;
        //     llog(DEBUG) << "pointInImage: " << pointInImage.x() << "," << pointInImage.y() << " | outerReference: " << outerReference.x() << "," << outerReference.y() << std::endl;
        // }
        isWhiteSpots.comparisons.push_back(comparison);
    }
    Vector2i innerReference = (pointInImage - n).cast<int>();
    if(innerReference.x() >= 0 && innerReference.x() < cameraInfo.width &&
            innerReference.y() >= 0 && innerReference.y() < cameraInfo.height) {
        if(isOuterPointInImage) {
            luminanceReference = (luminanceReference + ecImage.grayscaled[innerReference] + 1) / 2;
            saturationReference = (saturationReference + ecImage.saturated[innerReference]) / 2;
        }
        else {
            luminanceReference = ecImage.grayscaled[innerReference];
            saturationReference = ecImage.saturated[innerReference];
        }
        Comparison comparison;
        comparison.pointInImage = Spot(pointInImage);
        comparison.referencePoint = Spot(innerReference);
        comparison.type = 4;
        // if (comparison.hasZero()){
        //     llog(INFO) << "hasZero - 5 " << std::endl;
        //     llog(DEBUG) << "pointInImage: " << pointInImage.x() << "," << pointInImage.y() << " | referencePointInImage: " << innerReference.x() << "," << innerReference.y() << std::endl;

        // }
        isWhiteSpots.comparisons.push_back(comparison);
    }
    Vector2i intPointInImage = pointInImage.cast<int>();
    return relativeFieldColors.isWhiteNearField(ecImage.grayscaled[intPointInImage], ecImage.saturated[intPointInImage],
            static_cast<unsigned char>(luminanceReference), static_cast<unsigned char>(saturationReference));
}

float LineAndCircleMiddleInfo::calcWhiteCheckDistance(const Vector2f& pointOnField) const {
    if(pointOnField.squaredNorm() > squaredWhiteCheckNearField) {
        return whiteCheckDistance * (1.f + 0.5f * SQUARE(1.f - squaredWhiteCheckNearField / pointOnField.squaredNorm()));
    }
    else {
        return whiteCheckDistance;
    }
}

float LineAndCircleMiddleInfo::calcWhiteCheckDistanceInImage(const Vector2f& pointOnField, const CameraInfo& cameraInfo) const
{
    if(cameraInfo.camera == CameraInfo::Camera::bot)
    {
        return 15.f + 32.f * (1.f - (pointOnField.x() + 0.5f * std::abs(pointOnField.y())) / 1500.f);
    }
    else
    {
        // avoid computing a root. accurate enough for this purpose
        float yFactor = pointOnField.x() <= 0 ? 1.f : pointOnField.x() <= 350.f ? 0.25f + 0.75f * (1.f - pointOnField.x() / 350.f) : 0.25f;
        float estimatedDistance = pointOnField.x() + yFactor * std::abs(pointOnField.y());
        return 35000.f / estimatedDistance;
    }
}

int LineAndCircleMiddleInfo::previousVerticalScanLine(const Spot& spot, int scanLineId, 
        ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped) const {
    ASSERT(scanLineId >= 1);
    int previousScanLine = scanLineId - 1;
    if(highResolutionScan) {
        for(int i = 1; i <= 4 && scanLineId - i >= 0; ++i) {
            auto regions = colorScanLineRegionsVerticalClipped.scanLines[scanLineId - i].regions;
            if(regions.empty()) {
                continue;
            }
            if(static_cast<float>(regions[0].range.lower) >= spot.image.y()) {
                return scanLineId - i;
            }
        }
    }
    return previousScanLine;
}

bool LineAndCircleMiddleInfo::isSpotInsideObstacle(RobotObstaclesImage& robotsImage, const int fromX, const int toX, const int fromY, const int toY) const {   
    auto predicate = [&](const RobotObstaclesImage::Obstacle& obstacle) {
        if(toX >= obstacle.left && fromX <= obstacle.right && toY >= obstacle.top && fromY <= obstacle.bottom) {
            return true;
        }
        else {
            return false;
        }
    };

    return std::any_of(robotsImage.obstacles.cbegin(), robotsImage.obstacles.cend(), predicate);
}
