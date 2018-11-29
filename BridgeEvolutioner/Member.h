#ifndef MEMBER_H
#define MEMBER_H

#include "Joint.h"

struct Member {
    Joint first, second;
    double force = 0;
    //false = tension, true = compression
    bool force_type = false;
    bool calculated = false;

    bool check_intercept(const Member&);

    friend bool operator==(const Member&, const Member&);
    Member(Joint j1, Joint j2) : first(j1), second(j2) { };
    ~Member() { };
};

#endif // !MEMBER_H
