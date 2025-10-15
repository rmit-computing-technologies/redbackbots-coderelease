#pragma once

#include "types/math/Geometry.hpp"
#include "utils/math/LeastSquares.hpp"
#include <vector>

struct Spot {
    Vector2f image;
    Vector2f field;
    unsigned int candidate;

    Spot(const float imgX, const float imgY) : image(imgX, imgY) {}
    Spot(const Vector2f& image, const Vector2f& field) : image(image), field(field) {}
    Spot() : image(0, 0), field(0, 0), candidate(false) {}
    Spot(const Vector2i& image) : image(image.x(), image.y()) {}
    Spot(const Vector2f& image) : image(image.x(), image.y()) {}
};

struct Candidate {
    Vector2f n0;
    float d;
    std::vector<const Spot*> spots;

    Candidate(const Spot* anchor) : spots() {
        spots.emplace_back(anchor);
    }

    Candidate(){}

    /**
     * Calculates the distance of the given point to this line candidate.
     *
     * @param point point to calculate the distance to
     */
    float getDistance(const Vector2f& point) const {
        return std::abs(n0.dot(point) - d);
    }

    /**
     * Recalculates n0 and d.
     */
    void fitLine() {
        LeastSquares::LineFitter fitter;
        for(const Spot* spot : spots) {
            fitter.add(spot->field);
        }

        fitter.fit(n0, d);
    }
};



/**
 * @struct Candidates
 * This struct contains all the candidates for linespots
 */
class Candidates {
public:
    std::vector<Candidate> horizontalCandidates;
    std::vector<Candidate> verticalCandidates;
};



