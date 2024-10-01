#include "cam_provider.hpp"
//#include "ublox_provider.hpp" //Note: avoid package dependency, see CMakeList to enable
//#include "objects_provider.hpp" // Same here
#include "cpm_provider.hpp"
#include "denm_provider.hpp"
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "providers_launcher");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher publisher = nh.advertise<map_manager::MapObject>("map_manager", 1000);

    CamReceiver camReceiver(nh, nh_private,  publisher);
    //UbloxReceiver UbloxReceiver(nh, publisher);
    //ObjectsReceiver ObjectsReceiver(nh, publisher);
    CPMReceiver cpmReceiver(nh, publisher);
    DenmReceiver denmReceiver(nh, publisher);

    ros::spin();
    return 0;
}