#include "user_friendly_test_node/user_friendly_test.hpp"

int main(int argc, char **argv)
{
    Sailboat::UserTest controller("Test");
    controller.init(argc, argv);

    std::cout << "Testing all sensors" << std::endl;

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
