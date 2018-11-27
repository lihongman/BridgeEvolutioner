#ifndef BRIDGE_REQ_H
#define BRIDGE_REQ_H

#include <array>

#include "Joint.h"

const double MIN_WIDTH = 0;
const double MAX_WIDTH = 60;
const double MIN_LENGTH = 0;
const double MAX_LENGTH = 276;
const double MIN_HEIGHT = 0;
const double MAX_HEIGHT = 52.5;

const std::array<Joint, 3> req_side_joints = {
    Joint(0,0),
    Joint(0,276),
    Joint(0,246)
};

#endif // !BRIDGE_REQ_H
