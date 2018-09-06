#include "waypoint_follower_node/waypoint_follower.hpp"

int main(int argc, char **argv)
{
    Sailboat::WaypointFollower controller("WaypointFollower");
    controller.init(argc, argv);

    std::cout << "Begin Waypoint_Follower" << std::endl;

    while (ros::ok())
    {
		controller.loop();
    }
    
    
    return 0;
}
