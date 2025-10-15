#pragma once

class FieldFeature {
public:
    FieldFeature(double x, double y, double orientation)
        : x(x), y(y), orientation(orientation){};
    double x;
    double y;
    double orientation;
};
