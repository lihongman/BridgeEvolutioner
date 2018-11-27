#include <iostream>
#include "Bridge.h"

int main()
{
    Joint p1 = Joint(0, 0);
    Joint p2 = Joint(5, 0);
    Joint p3 = Joint(0, 2);
    Joint p4 = Joint(0, 0);

    Bridge bridge = Bridge();

    bridge.add_joint(p1);
    bridge.add_joint(p2);
    bridge.add_joint(p3);
    if (true)
    {
        std::cout << "Success" << std::endl;
    }
    else
    {
        std::cout << "Fail" << std::endl;
    }

    std::cin.get();
    return 0;
}