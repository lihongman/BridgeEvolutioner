#include <algorithm>
#include <queue>
#include <memory>
#include <vector>
#include <cmath>
#include <ctime>
#include <sstream>

#include <iostream>

#include "Bridge.h"

Bridge::~Bridge()
{
    members.clear();
    joints.clear();
    joint_member_list.clear();
}

double Bridge::weight()
{
    double output = 0;
    for (std::shared_ptr<Member> member : members)
    {
        output += member->length() * MATERIAL_WEIGHT;
    }
    return output * 2;
}

bool Bridge::add_joint(Joint& joint)
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

void Bridge::add_vertical_loads(int loading, bool side)
{
    double L1, L2;

    if (loading < 0)
    {
        if (side)
        {
            L1 = 1;
            L2 = 0;
        }
        else
        {
            L2 = 1;
            L1 = 0;
        }
    }
    else
    {
        L1 = L1_LOAD;
        L2 = L2_LOAD;
    }

    joints.back()->load = -(L1 * load_locations[abs(loading)-1][0] + L2 * load_locations[abs(loading)-1][1]) / MAX_LENGTH;
    joints.front()->load = -(L1 + L2) - joints.back()->load;
    std::shared_ptr<Joint> last_joint = joints.front();
    std::vector<std::shared_ptr<Joint>> list;
    for (std::shared_ptr<Joint> joint : joints)
    {
        if (last_joint->y == 0 && joint->y == 0)
        {
            list.push_back(joint);
        }
    }
    std::sort(list.begin(), list.end(), [](std::shared_ptr<Joint> a, std::shared_ptr<Joint> b)
    {
        return a->x < b->x;
    });
    for (std::shared_ptr<Joint> joint : joints)
    {
        double distance = joint->x - last_joint->x;
        if (joint->x == load_locations[abs(loading) - 1][0])
            joint->load += L1;
        if (joint->x == load_locations[abs(loading) - 1][1])
            joint->load += L2;
        if (last_joint->x < load_locations[abs(loading)-1][0] && joint->x > load_locations[abs(loading)-1][0])
        {
            double load = L1 * (load_locations[abs(loading)-1][0] - last_joint->x) / distance;
            double oload = L1 - load;
            last_joint->load += load;
            joint->load += oload;
        }
        if (last_joint->x < load_locations[abs(loading)-1][1] && joint->x > load_locations[abs(loading)-1][1])
        {
            double load = L2 * (load_locations[abs(loading)-1][1] - last_joint->x) / distance;
            double oload = L2 - load;
            last_joint->load += load;
            joint->load += oload;
        }
        last_joint = joint;
    }
}

double solve_member(double forcex, double forcey, const Member& m1, const Member& m2, const Joint& joint)
{
    Joint j1;
    Joint j2;
    if (*m1.first == joint) { j1 = *m1.second; }
    else { j1 = *m1.first; }
    if (*m2.first == joint) { j2 = *m2.second; }
    else { j2 = *m2.first; }

    double angle1 = atan2(j1.y - joint.y, j1.x - joint.x);
    double angle2 = atan2(j2.y - joint.y, j2.x - joint.x);

    return (-forcex * sin(angle1) + forcey * cos(angle1)) / ((sin(angle2) * cos(angle1)) - (sin(angle1) * cos(angle2)));
}

void calculate_joint(const Joint& joint, std::list<std::shared_ptr<Member>>& member_list)
{
    double forcex = 0;
    double forcey = -joint.load;
    std::vector<std::shared_ptr<Member>> temp;
    for (std::shared_ptr<Member> member : member_list)
    {
        if (member->calculated)
        {
            Joint second;
            if (*member->first == joint)
            {
                second = *member->second;
            }
            else
            {
                second = *member->first;
            }
            if (joint.x < second.x) { 
                if (member->force_type)
                    forcex -= member->forcex;
                else
                    forcex += member->forcex;
            }
            else 
            { 
                if (member->force_type)
                    forcex += member->forcex;
                else
                    forcex -= member->forcex;
            }
            if (joint.y < second.y) {
                if (member->force_type)
                    forcey -= member->forcey;
                else
                    forcey += member->forcey;
            }
            else
            {
                if (member->force_type)
                    forcey += member->forcey;
                else
                    forcey -= member->forcey;
            }
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
        if (forcex > 0 || forcey > 0)
        {
            temp[0]->force_type = true;
        }
        else
        {
            temp[0]->force_type = false;
        }
    }
}

bool Bridge::method_of_joints()
{
    std::shared_ptr<Joint> temp;
    std::queue<std::shared_ptr<Joint>> jqueue;
    jqueue.push(joints.front());
    int iterations = 0;
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
                ++iterations;
                if (iterations > 1000)
                {
                    return false;
                }
                jqueue.push(temp);
            }
        }
    }
    return true;
}

void Bridge::reset_bridge_load()
{
    for (std::shared_ptr<Joint> joint : joints)
    {
        joint->calculated = false;
        joint->load = 0;
    }
    for (std::shared_ptr<Member> member : members)
    {
        member->calculated = false;
    }
}

long double Bridge::vertical_deflection()
{
    long double output;
    long double max = 0;
    reset_bridge_load();
    for (unsigned int j = 1; j < load_locations.size() + 1; j++)
    {
        int i = 0;
        output = 0;
        //Gets first 
        add_vertical_loads(-1 * j, true);
        method_of_joints();
        for (std::shared_ptr<Member> member : members)
        {
            unit_members1.push_back(member->force);
            if (member->force_type)
                unit_members1[i] *= -1;
            i++;
        }
        reset_bridge_load();
        i = 0;
        add_vertical_loads(-1 * j, false);
        method_of_joints();
        for (std::shared_ptr<Member> member : members)
        {
            unit_members2.push_back(member->force);
            if (member->force_type)
                unit_members2[i] *= -1;
            i++;
        }
        reset_bridge_load();
        i = 0;
        add_vertical_loads(j, false);
        if (!method_of_joints())
            return -100;
        for (std::shared_ptr<Member> member : members)
        {
            output += (unit_members1[i] * (member->force / 1000) * member->length()) / (MEMBER_AREA * YOUNGS_MODULUS);
            output += (unit_members2[i] * (member->force / 1000) * member->length()) / (MEMBER_AREA * YOUNGS_MODULUS);
            i++;
        }
        if (max < output)
        {
            max = output;
        }
    }
    unit_members1.clear();
    unit_members2.clear();

    deflection = max;
    if (max == 0)
        max = 0;
    return max;
}

void Bridge::mutate(std::shared_ptr<Bridge> bridge)
{
    members.clear();
    joints.clear();
    joint_member_list.clear();
    int jointed = 0;
    for (std::shared_ptr<Joint> joint : bridge->joints)
    {
        add_joint(*joint);
        jointed++;
    }
    for (std::shared_ptr<Member> member : bridge->members)
    {
        add_member(*member->first, *member->second);
    }
    bool valid = true;

    std::normal_distribution<double> distribution(0, 0.5);

    for (std::shared_ptr<Joint> joint : joints)
    {
        bool required = false;
        std::list<std::shared_ptr<Member>> temp_list = joint_member_list[*joint];
        joint_member_list.erase(*joint);
        for (Joint rjoint : req_side_joints)
        {
            if (rjoint == *joint)
            {
                required = true;
            }
        }
        if (!required)
        {
            int count;
            do {
                required = false;
                double modifyx = distribution(generator) / 2;
                double modifyy = distribution(generator) / 2;
                modifyx = floor((modifyx * 2) + 0.5) / 2;
                modifyy = floor((modifyy * 2) + 0.5) / 2;
                if (joint->y == 0)
                {
                    joint->x += modifyx;
                }
                else
                {
                    joint->x += modifyx;
                    joint->y += modifyy;
                    if (joint->y == 0)
                    {
                        while (joint->y <= 0)
                        {
                            joint->y += 0.5;
                        }
                    }
                }

                count = std::count_if(joints.begin(), joints.end(), [joint](std::shared_ptr<Joint> j) { return *j == *joint; });

                for (Joint rjoint : req_side_joints)
                {
                    if (rjoint == *joint)
                    {
                        required = true;
                    }
                }

            } while (required && count > 1);
        }
        joint_member_list[*joint] = temp_list;
    }
}

bool Bridge::validate()
{
    unsigned int num_joints = joints.size();
    unsigned int num_members = members.size();
    std::unique(joints.begin(), joints.end(), [](const std::shared_ptr<Joint>& j1, const std::shared_ptr<Joint>& j2) { return *j1 == *j2; });
    if (num_joints > joints.size())
        return false;
    std::unique(members.begin(), members.end(), [](const std::shared_ptr<Member>& j1, const std::shared_ptr<Member>& j2) { return *j1 == *j2; });
    if (num_members > members.size())
        return false;
    for (std::shared_ptr<Joint> joint : joints)
    {
        if (!joint->check(MIN_LENGTH, MAX_LENGTH, MIN_HEIGHT, MAX_HEIGHT))
        {
            return false;
        }
    }
    for (std::shared_ptr<Member> member : members)
    {
        if (*member->first == *member->second)
            return false;
        for (std::shared_ptr<Member> amember : members)
        {
            if (member->check_intercept(*amember) && !(*member == *amember))
            {
                return false;
            }
        }
    }
    return true;
}

std::string Bridge::print()
{
    std::stringstream stream;
    stream << "Joints\n\n";
    for (std::shared_ptr<Joint> joint : joints)
    {
        stream << '(' << joint->x << ',' << joint->y << ")\n";
    }
    stream << "\n\n\nMembers\n";
    for (std::shared_ptr<Member> member : members)
    {
        stream << '(' << member->first->x << ',' << member->first->y << ") , " << '(' << member->second->x << ',' << member->second->y << ")\n";
    }
    return stream.str();
}