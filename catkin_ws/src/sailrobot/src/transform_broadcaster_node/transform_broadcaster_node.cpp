#include "transform_broadcaster_node/transform_broadcaster.hpp"

int main(int argc, char **argv)
{
    Sailboat::TransformBroadcaster controller("Transform_Broadcaster");
    controller.init(argc, argv);

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
