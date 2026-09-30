/**
 * @file Path.hpp
 * @brief path
 */

#include <vector>
#include "Point.hpp"

using namespace std;

class Path {
public:
    vector<Point> path_dots;
    Path(vector<Point> dots): path_dots(dots) {}
};

