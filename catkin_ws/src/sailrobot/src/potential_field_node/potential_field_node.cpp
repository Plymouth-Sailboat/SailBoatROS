#include "potential_field_node/potentialfield.hpp"

int main(int argc, char **argv)
{
    Sailboat::PotentialField controller("Controller", 10);
    controller.init(argc, argv);

    std::cout << "Began Potential_field" << std::endl;

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
