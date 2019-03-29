#include "potential_field_node/potential_field.hpp"

int main(int argc, char **argv)
{
    Sailboat::PotentialField controller("Controller");
    controller.init(argc, argv);

    std::cout << "Begin Potential_field" << std::endl;

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
