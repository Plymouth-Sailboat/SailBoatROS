#include "identification_node/identification.hpp"

int main(int argc, char **argv)
{
    Sailboat::Identification controller("Controller");
    controller.init(argc, argv);

    std::cout << "Begin Identification" << std::endl;

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
