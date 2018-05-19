#include "area_scanning_node/area_scanning.hpp"

int main(int argc, char **argv)
{
    Sailboat::AreaScanning controller("AreaScanning");
    controller.init(argc, argv);

    std::cout << "Begin Area_scanning" << std::endl;

    while (ros::ok())
    {
		controller.loop();
    }
    
    
    return 0;
}
