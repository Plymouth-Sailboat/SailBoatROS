#include "line_following_long_node/line_following_long.hpp"

int main(int argc, char **argv)
{
    Sailboat::LineFollowingLong controller("LineFollowing");
    controller.init(argc, argv);

    std::cout << "Begin Line_Following" << std::endl;

    while (ros::ok())
    {
		controller.loop();
    }
    
    
    return 0;
}
