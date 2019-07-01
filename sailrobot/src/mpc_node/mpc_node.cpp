#include "mpc_node/mpc.hpp"

int main(int argc, char **argv)
{
    Sailboat::MPC controller("MPC");
    controller.init(argc, argv);

    std::cout << "Begin MPC" << std::endl;

    while (ros::ok())
    {
	controller.loop();
    }
    
    
    return 0;
}
