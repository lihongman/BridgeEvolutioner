#ifndef JOINT_H
#define JOINT_H

struct Joint {
    double x, y, load, reaction;
    bool calculated = false;
    bool check(double min_x, double max_x, double min_y, double max_y);

    friend bool operator==(const Joint&, const Joint&);

    Joint(double x, double y) : x(x), y(y), load(0), reaction(0) { };
    Joint() : x(0), y(0), load(0), reaction(0) { };
    ~Joint() { };
};

struct JointHash {
    size_t operator()(const Joint&) const;
};

#endif // !JOINT_H
