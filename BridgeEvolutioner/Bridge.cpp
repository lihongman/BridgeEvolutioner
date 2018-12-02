#include <algorithm>
#include <queue>
#include <memory>

#include "Bridge.h"

Bridge::~Bridge()
{
    members.clear();
    joints.clear();
    joint_member_list.clear();
}

bool Bridge::add_joint(Joint& joint)
{
    if (joint.check(MIN_LENGTH, MAX_LENGTH, MIN_HEIGHT, MAX_HEIGHT))
    {
        if (joints.size() == 0)
        {
            joints.push_back(std::make_shared<Joint>(joint.x, joint.y, joint.load));
            return true;
        }
        std::list<std::shared_ptr<Joint>>::iterator it = std::find_if(joints.begin(), joints.end(),
            [joint](std::shared_ptr<Joint> const& j) { return *j == joint; });
        if (it == joints.end())
        {
            joints.push_back(std::make_shared<Joint>(joint.x, joint.y, joint.load));
            return true;
        }
    }
    return false;
}

bool Bridge::add_member(Joint& first, Joint& second)
{
    std::list<std::shared_ptr<Joint>>::iterator itfirst = std::find_if(joints.begin(), joints.end(),
        [first](std::shared_ptr<Joint> const& j) { return *j == first; });
    std::list<std::shared_ptr<Joint>>::iterator itsecond = std::find_if(joints.begin(), joints.end(),
        [second](std::shared_ptr<Joint> const& j) { return *j == second; });
    if (members.size() == 0)
    {
        if (itfirst != joints.end() && itsecond != joints.end())
        {
            members.push_back(std::make_shared<Member>(*itfirst, *itsecond));
            return true;
        }
        return false;
    }
    std::list<std::shared_ptr<Member>>::iterator it = std::find_if(members.begin(), members.end(),
        [first, second](std::shared_ptr<Member> const& m) { 
        return (*(m->first) == first && *(m->second) == second) || (*(m->first) == second && *(m->second) == first);
    });
    if (itfirst != joints.end() && itsecond != joints.end() && it == members.end())
    {
        std::shared_ptr<Member> member = std::make_shared<Member>(*itfirst, *itsecond);
        members.push_back(member);
        joint_member_list[first].push_back(member);
        joint_member_list[second].push_back(member);
        return true;
    }
    return false;
}

bool Bridge::remove_joint(Joint& joint)
{
    int count = 0;
    //Checks if joint is one of required joints.
    for (Joint i : req_side_joints)
    {
        if (joint == i)
        {
            return false;
        }
    }
    //Removes joint only if number of members are less than three.
    if (joint_member_list[joint].size() < 3)
    {
        joint_member_list.erase(joint);
        joints.remove_if([joint](std::shared_ptr<Joint> const& j) { return *j == joint; });
        members.remove_if([joint](std::shared_ptr<Member> const& n) { return *(n->first) == joint || *(n->second) == joint; });
        return true;
    }
    return false;
}

void Bridge::remove_member(Joint& first, Joint& second)
{
    joint_member_list[first].remove_if([first, second](std::shared_ptr<Member> const& m) { return (*(m->first) == first && *(m->second) == second); });
    joint_member_list[second].remove_if([first, second](std::shared_ptr<Member> const& m) { return (*(m->first) == first && *(m->second) == second); });
    members.remove_if([first, second](std::shared_ptr<Member> const& m) { return (*(m->first) == first && *(m->second) == second); });
}

bool Bridge::stable_determinate()
{
    int unknowns = 3 + members.size();
    int equations = 2 * joints.size();
    return unknowns == equations;
}

void method_of_joints_optimize()
{
    std::queue<Joint> joint_list;
}

double Bridge::deflection(int version)
{
    switch (version)
    {
        case 1:
            method_of_joints_optimize();
            break;
    }
    return 2;
}