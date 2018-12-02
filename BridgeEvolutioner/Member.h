#ifndef MEMBER_H
#define MEMBER_H

#include <memory>

#include "Joint.h"

struct Member {
    std::shared_ptr<Joint> first, second;
    double force = 0;
    bool calculated = false;

    bool check_intercept(const Member&);

    friend bool operator==(const Member&, const Member&);
    Member(const std::shared_ptr<Joint>& j1, const std::shared_ptr<Joint>& j2) : first(j1), second(j2) { };
    Member(const Member&);
    ~Member() { };
};

#endif // !MEMBER_H
