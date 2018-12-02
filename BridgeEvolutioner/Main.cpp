#define _CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>
#include <iostream>

#include "Bridge.h"

int main()
{
    Joint A = Joint(0, 0);
    Joint B = Joint(4, 5);
    Joint C = Joint(4, 0);
    Joint D = Joint(8, 4);
    Joint E = Joint(8, 0);
    Joint F = Joint(11, 0);

    std::shared_ptr<Bridge> bridge = std::make_shared<Bridge>();

    bridge->add_joint(A);
    bridge->add_joint(B);
    bridge->add_joint(C);
    bridge->add_joint(D);
    bridge->add_joint(E);
    bridge->add_joint(F);

    bridge->add_member(A, B);
    bridge->add_member(A, C);
    bridge->add_member(B, C);
    bridge->add_member(B, D);
    bridge->add_member(C, D);
    bridge->add_member(C, E);
    bridge->add_member(D, E);
    bridge->add_member(D, F);
    bridge->add_member(E, F);

    bridge->vertical_deflection();

    if (bridge->add_member(B, C))
    {
        std::cout << "Success" << std::endl;
    }
    else
    {
        std::cout << "Fail" << std::endl;
    }

    bridge.reset();

    std::shared_ptr<Joint> j1 = std::make_shared<Joint>(0, 0);
    std::shared_ptr<Joint> j2 = std::make_shared<Joint>(-3, 5);
    Member temp = Member(j1, j2);
    temp.set_force(50);

    _CrtDumpMemoryLeaks();
    std::cin.get();
    return 0;
}