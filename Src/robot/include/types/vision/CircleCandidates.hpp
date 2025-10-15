#pragma once

#include "types/math/Geometry.hpp"
#include "utils/math/LeastSquares.hpp"
#include "types/vision/Candidates.hpp"
#include <vector>


/**
 * Structure for a center circle candidate.
 */
struct CircleCandidate {
    Vector2f center;
    float radius;
    std::vector<Vector2f> fieldSpots;
    std::vector<Vector2f> imageSpots;
    LeastSquares::CircleFitter fitter;

    CircleCandidate(const Candidate& line, const Vector2f& fieldSpot, const Vector2f& imageSpot) {
        for(const Spot* const lineSpot : line.spots) {
            fieldSpots.emplace_back(lineSpot->field);
            imageSpots.emplace_back(lineSpot->image);
        }
        fieldSpots.emplace_back(fieldSpot);
        imageSpots.emplace_back(imageSpot);
        fitter.add(fieldSpots);
        if(!fitter.fit(center, radius)) {
            radius = std::numeric_limits<float>::max();
        }
    }

    CircleCandidate(): center(0,0), radius(0), fieldSpots(), imageSpots() {}

    /**
     * Adds the given field spot to the candidate and refits the circle.
     *
     * @param spot field spot to add
     */
    void addSpot(const Vector2f& fieldSpot, const Vector2f& imageSpot) {
        fieldSpots.emplace_back(fieldSpot);
        imageSpots.emplace_back(imageSpot);
        fitter.add(fieldSpot);
        if(!fitter.fit(center, radius)) {
            radius = std::numeric_limits<float>::max();
        }
    }

    /**
     * Calculates the distance of the given point to this circle candidate.
     *
     * @param point point to calculate the distance to
     */
    float getDistance(const Vector2f& point) const {
        return std::abs((center - point).norm() - radius);
    }

    /**
     * Calculates the average error of this circle candidate.
     */
    float calculateError() const {
        float error = 0.f;
        for(const Vector2f& spot : fieldSpots) {
            error += getDistance(spot);
        }
        return error / static_cast<float>(fieldSpots.size());
    }

    /**
     * Calculates how much of a circle corresponding to this candidate lies between the outermost fieldSpots.
     * @return Portion of the candidate that is between the outermost fieldSpots expressed as an angle.
     */
    Angle circlePartInImage() const {
        Vector2f referenceVector = fieldSpots[0] - center;
        Angle low = 0_deg;
        Angle high = 0_deg;
        auto spot = fieldSpots.cbegin();
        ++spot;
        for(; spot != fieldSpots.cend(); ++spot) {
            Vector2f spotAngleVector = *spot - center;
            Angle spotToReference = spotAngleVector.angleTo(referenceVector);
            if(spotAngleVector.x()*referenceVector.y() - spotAngleVector.y()*referenceVector.x() < 0) {
                if(spotToReference > low) {
                    low = spotToReference;
                }
            }
            else {
                if(spotToReference > high) {
                    high = spotToReference;
                }
            }
        }
        return low + high;
    }

    /**
     * Computes the average distance of the robot to all field spots.
     * The distance will be used later for computing a covariance of the circle measurement.
     *
     * @param The average distance to all points that form the circle
     */
    float getAverageDistanceToFieldSpots() {
        float sqrDistSum = 0.f;
        for(const Vector2f& spot : fieldSpots) {
            sqrDistSum += spot.squaredNorm();
        }
        return std::sqrt(sqrDistSum / static_cast<float>(fieldSpots.size())); // Function will/should not be called, if there are no spots.
    }
};

class CircleCandidates {
    public:
        std::vector<CircleCandidate, Eigen::aligned_allocator<CircleCandidate>> candidates;
};
