#ifndef BRIDGE_H
#define BRIDGE_H

#include <list>
#include <unordered_map>
#include <memory>
#include <vector>

#include "Bridge_Req.h"
#include "Member.h"
#include "Joint.h"

class Bridge {
private:
    std::shared_ptr<Joint> start;
    std::list<std::shared_ptr<Member>> members;
    std::list<std::shared_ptr<Joint>> joints;
    std::unordered_map<Joint, std::list<std::shared_ptr<Member>>, JointHash> joint_member_list;

    std::vector<Member> unit_members;

    void reset_bridge_load();
    void method_of_joints();
    void remove_member(Joint&, Joint&);
    bool remove_joint(Joint&);
public:
    bool stable_determinate();
    double vertical_deflection();
    double weight();
    void mutate(std::shared_ptr<Bridge>);
    bool add_member(Joint&, Joint&);
    bool add_joint(Joint&);

    Bridge() { };
    ~Bridge();
};
#endif // !BRIDGE_H
