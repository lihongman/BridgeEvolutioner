#include <string>

#include "Joint.h"

Joint::Joint(const Joint& j)
{
    x = j.x;
    y = j.y;
    load = j.load;
}

bool Joint::check(double min_x, double max_x, double min_y, double max_y)
{
    return (x >= min_x) && (x <= max_x) && (y >= min_y) && (y <= max_y);
}

bool operator==(const Joint& j1, const Joint& j2)
{
    return j1.x == j2.x && j1.y == j2.y;
}

size_t JointHash::operator()(const Joint& k) const
{
    std::string output = std::to_string(k.x) + ',' + std::to_string(k.y);
    return std::hash<std::string>{}(output);
}