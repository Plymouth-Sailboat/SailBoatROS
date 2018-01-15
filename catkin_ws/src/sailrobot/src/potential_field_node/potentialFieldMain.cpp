#include "potential_field_node/potentialfield.hpp"

int main(int argc, char **argv)
{
    Sailboat::PotentialField controller("Controller", 10, 0);
    controller.init(argc, argv);
   
    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
