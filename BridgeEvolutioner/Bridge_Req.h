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

const double L1_LOAD = 1400/2;
const double L2_LOAD = 900/2;
const double LAT_LOAD = 75;

const double YOUNGS_MODULUS = 30000;
const double MEMBER_AREA = 0.2431;
const double MATERIAL_WEIGHT = 0.069;

const std::array<Joint, 3> req_side_joints = {
    Joint(0,0),
    Joint(36,0),
    Joint(264,0)
};

const std::array<std::array<int, 3>, 6> load_locations = {
    std::array<int, 3>{132, 72, 138},
    std::array<int, 3>{144, 96, 138},
    std::array<int, 3>{162, 120, 0},
    std::array<int, 3>{168, 100, 0},
    std::array<int, 3>{174, 128, 0},
    std::array<int, 3>{185, 125, 138}
};

#endif // !BRIDGE_REQ_H
