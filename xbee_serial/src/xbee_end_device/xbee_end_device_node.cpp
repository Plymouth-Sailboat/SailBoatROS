#include "xbee_end_device/xbee_end_device.hpp"

int main(int argc, char **argv)
{
    Xbee::XbeeEndDevice controller("XbeeEndDevice");
    controller.init(argc, argv);

    while (ros::ok())
    {
        controller.loop();
    }


    return 0;
}

