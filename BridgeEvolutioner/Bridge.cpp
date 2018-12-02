#include <algorithm>
#include <queue>
#include <memory>
#include <vector>

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
            start = std::make_shared<Joint>(joint.x, joint.y);
            joints.push_back(start);
            return true;
        }
        std::list<std::shared_ptr<Joint>>::iterator it = std::find_if(joints.begin(), joints.end(),
            [joint](std::shared_ptr<Joint> const& j) { return *j == joint; });
        if (it == joints.end())
        {
            joints.push_back(std::make_shared<Joint>(joint.x, joint.y));
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
            std::shared_ptr<Member> member = std::make_shared<Member>(*itfirst, *itsecond);
            members.push_back(member);
            joint_member_list[first].push_back(member);
            joint_member_list[second].push_back(member);
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

void add_vertical_loads()
{

}

double solve_member(double forcex, double forcey, const Member& m1, const Member& m2, const Joint& joint)
{
    Joint j1;
    Joint j2;
    if (*m1.first == joint) { j1 = *m1.second; } else { j1 = *m1.first; }
    if (*m2.first == joint) { j2 = *m2.second; } else { j2 = *m2.first; }

    return 0;
}

void calculate_joint(const Joint& joint, std::list<std::shared_ptr<Member>>& member_list)
{
    double forcex = 0;
    double forcey = joint.reaction - joint.load;
    std::vector<std::shared_ptr<Member>> temp;
    for (std::shared_ptr<Member> member : member_list)
    {
        if (member->calculated)
        {
            Joint first;
            Joint second;
            if (*member->first == joint)
            {
                first = *member->first;
                second = *member->second;
            }
            else
            {
                second = *member->first;
                first = *member->second;
            }
            if (first.x < second.x) { forcex -= member->forcex; }
            else { forcex += member->forcex; }
            if (first.y < second.y) { forcey -= member->forcey; }
            else { forcey += member->forcey; }
        }
        else
        {
            temp.push_back(member);
            member->calculated = true;
        }
    }
    if (temp.size() == 2)
    {
        temp[0]->set_force(solve_member(forcex, forcey, *temp[1], *temp[0], joint));
        temp[1]->set_force(solve_member(forcex, forcey, *temp[0], *temp[1], joint));
    }
    if (temp.size() == 1)
    {
        temp[0]->force = sqrt(pow(forcex, 2) + pow(forcey, 2));
        temp[0]->forcex = abs(forcex);
        temp[0]->forcey = abs(forcey);
    }
}

void Bridge::method_of_joints()
{
    std::shared_ptr<Joint> temp;
    std::queue<std::shared_ptr<Joint>> jqueue;
    jqueue.push(start);
    while (!jqueue.empty())
    {
        temp = jqueue.front();
        jqueue.pop();
        if (!temp->calculated)
        {
            int count = std::count_if(joint_member_list[*temp].begin(), joint_member_list[*temp].end(),
                [](std::shared_ptr<Member> m) { return !m->calculated; });
            if (count < 3)
            {
                for (std::shared_ptr<Member> member : joint_member_list[*temp])
                {
                    if (!member->calculated)
                    {
                        if (*member->first == *temp)
                        {
                            jqueue.push(member->second);
                        }
                        else
                        {
                            jqueue.push(member->first);
                        }
                    }
                }
                calculate_joint(*temp, joint_member_list[*temp]);
                temp->calculated = true;
            }
            else
            {
                jqueue.push(temp);
            }
        }
    }
}

double Bridge::vertical_deflection()
{
    add_vertical_loads();
    method_of_joints();
    return 2;
}