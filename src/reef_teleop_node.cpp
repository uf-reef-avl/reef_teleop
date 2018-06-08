#include <ros/ros.h>

#include "Teleop.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "reef_teleop");
    reef_teleop::Teleop teleop_object;
    ros::spin();

    return 0;
}
