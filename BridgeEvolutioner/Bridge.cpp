#include <algorithm>
#include <queue>

#include "Bridge.h"

bool Bridge::add_joint(Joint& joint)
{
    if (joint.check(MIN_LENGTH, MAX_LENGTH, MIN_HEIGHT, MAX_HEIGHT))
    {
        if (joints.size() == 0)
        {
            joints.push_back(joint);
            return true;
        }
        std::list<Joint>::iterator it = std::find(joints.begin(), joints.end(), joint);
        if (it == joints.end())
        {
            joints.push_back(joint);
            return true;
        }
    }
    return false;
}

bool Bridge::add_member(Member& member)
{
    if (members.size() == 0)
    {
        members.push_back(member);
        return true;
    }
    std::list<Member>::iterator it = std::find(members.begin(), members.end(), member);
    if (it == members.end())
    {
        members.push_back(member);
        joint_member_list[member.first].push_back(members);
        joint_member_list[member.second].push_back(member);
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
        joints.remove(joint);
        members.remove_if([&joint](Member n) { return n.first == joint || n.second == joint; });
        return true;
    }
    return false;
}

void Bridge::remove_member(Member& member)
{
    joint_member_list[member.first].remove(member);
    joint_member_list[member.second].remove(member);
    members.remove(member);
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