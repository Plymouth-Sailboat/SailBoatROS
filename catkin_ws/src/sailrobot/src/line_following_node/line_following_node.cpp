#include "line_following_node/line_following.hpp"

int main(int argc, char **argv)
{
    Sailboat::LineFollowing controller("LineFollowing");
    controller.init(argc, argv);

    std::cout << "Begin Line_Following" << std::endl;

    while (ros::ok())
    {
		controller.loop();
    }
    
    
    return 0;
}
