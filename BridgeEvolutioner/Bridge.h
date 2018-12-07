#ifndef BRIDGE_H
#define BRIDGE_H

#include <list>
#include <unordered_map>
#include <memory>
#include <vector>
#include <random>
#include <string>

#include "Bridge_Req.h"
#include "Member.h"
#include "Joint.h"

class Bridge {
private:
    std::list<std::shared_ptr<Member>> members;
    std::list<std::shared_ptr<Joint>> joints;
    std::unordered_map<Joint, std::list<std::shared_ptr<Member>>, JointHash> joint_member_list;
    std::default_random_engine generator;

    std::vector<double> unit_members1;
    std::vector<double> unit_members2;

    void reset_bridge_load();
    bool method_of_joints();
    void remove_member(Joint&, Joint&);
    bool remove_joint(Joint&);
    void add_vertical_loads(int, bool);
public:
    long double fitness = 0;
    long double deflection = 0;
    
    bool stable_determinate();
    long double vertical_deflection();
    double weight();
    void mutate(std::shared_ptr<Bridge>);
    bool validate();
    bool add_member(Joint&, Joint&);
    bool add_joint(Joint&);
    std::string print();

    Bridge() 
    {
        generator.seed((unsigned int)time(NULL));
    };
    ~Bridge();
};
#endif // !BRIDGE_H
