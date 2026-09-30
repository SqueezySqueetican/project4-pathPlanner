#pragma once
#include <list>

struct Point
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& point) {
        return (x != point.x || y != point.y);
    }
};