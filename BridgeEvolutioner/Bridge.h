#ifndef BRIDGE_H
#define BRIDGE_H

#include <list>
#include <unordered_map>

#include "Bridge_Req.h"
#include "Member.h"
#include "Joint.h"

class Bridge {
private:
    std::list<Member> members;
    std::list<Joint> joints;
    std::unordered_map<Joint, std::list<Member>, JointHash> joint_member_list;

    void remove_member(Member&);
    bool remove_joint(Joint&);
public:
    bool stable_determinate();
    double deflection();
    void mutate();
    bool add_member(Member&);
    bool add_joint(Joint&);

    Bridge() { };
    ~Bridge() { };
};
#endif // !BRIDGE_H
