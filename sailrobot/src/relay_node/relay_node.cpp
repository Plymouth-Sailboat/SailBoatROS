#include "relay_node/relay.hpp"

int main(int argc, char **argv)
{
    Sailboat::Relay controller("Relay");
    controller.init(argc, argv);

    std::cout << "Begin Relay Controller" << std::endl;

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
