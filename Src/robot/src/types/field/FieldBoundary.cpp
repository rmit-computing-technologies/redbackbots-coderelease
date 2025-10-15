
#include "types/field/FieldBoundary.hpp"

#include "utils/debug/Assert.hpp"

FieldBoundary::FieldBoundary() :
    isValid(false),
    extrapolated(false),
    odd(false)
{}

int FieldBoundary::getBoundaryY(int x) const {
    ASSERT(boundaryInImage.size() >= 2);

    const Vector2i* left = &boundaryInImage.front();
    const Vector2i* right = &boundaryInImage.back();

    if(x < left->x()) {
        right = &(*(boundaryInImage.begin() + 1));
    } else if(x > right->x()) {
        left = &(*(boundaryInImage.end() - 2));
    } else {
        for(const Vector2i& point : boundaryInImage) {
            if(point.x() == x) {
                return point.y();
            } else if(point.x() < x && point.x() > left->x()) {
                left = &point;
            } else if(point.x() > x && point.x() < right->x()) {
                right = &point;
            }
        }
    }

    float m = static_cast<float>(right->y() - left->y()) / static_cast<float>(right->x() - left->x());

    return static_cast<int>(static_cast<float>(x - left->x()) * m) + left->y();
}

int FieldBoundary::getBoundaryTopmostY(int imageWidth) const {
    if(!isValid) {
        return 0;
    }

    int topmostY = std::min(getBoundaryY(0), getBoundaryY(imageWidth));
    for(const auto& boundarySpot : boundaryInImage) {
        if(boundarySpot.y() < topmostY) {
            topmostY = boundarySpot.y();
        }
    }
    
    return topmostY;
}

void FieldBoundary::reset() {
    boundaryOnField.clear();
    boundaryInImage.clear();
    boundaryInImageLowerBound.clear();
    boundaryInImageUpperBound.clear();
    isValid = false;
    extrapolated = false;
    odd = false;
}
