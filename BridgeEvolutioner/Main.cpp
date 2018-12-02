#define _CRTDBG_MAP_ALLOC  
#include <stdlib.h>  
#include <crtdbg.h>
#include <iostream>

#include "Bridge.h"

int main()
{
    Joint p1 = Joint(0, 0);
    Joint p2 = Joint(5, 0);
    Joint p3 = Joint(0, 2);
    //Joint p4 = Joint(3, 4);

    std::shared_ptr<Bridge> bridge = std::make_shared<Bridge>();

    bridge->add_joint(p1);
    bridge->add_joint(p2);
    bridge->add_joint(p3);

    bridge->add_member(p1, p2);
    bridge->add_member(p2, p3);
    bridge->add_member(p1, p3);
    //bridge->add_member(p2, p1);

    if (bridge->add_member(p2, p1))
    {
        std::cout << "Success" << std::endl;
    }
    else
    {
        std::cout << "Fail" << std::endl;
    }

    bridge.reset();

    _CrtDumpMemoryLeaks();
    std::cin.get();
    return 0;
}