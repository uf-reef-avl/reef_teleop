#ifndef REEF_TELEOP_TELEOP_H
#define REEF_TELEOP_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <rosflight_msgs/RCRaw.h>
#include <reef_msgs/DesiredState.h>

namespace reef_teleop {
    struct Axis
    {
        Axis() : axis(0), max(0.0), offset(0.0) {}

        int axis;
        double max;
        double offset;
    };

    class Teleop {
    private:
        ros::NodeHandle private_nh_;
        ros::NodeHandle node_handle_;

        ros::Subscriber joy_subscriber_;
        ros::Publisher desired_state_publisher_;

        struct {
            Axis x;
            Axis y;
            Axis z;
            Axis thrust;
            Axis yaw;
        } axes;

        struct {
            int a;
            int b;
            int x;
            int y;
        } buttons;

        double zCommand, zCommandMin, zCommandMax, zScale;
        bool invertX;
        bool rcSwitchFlipped;
        int rcChannel;
        std::string control_mode;

    public:
        Teleop();
        void joyAttAltCallback(const sensor_msgs::JoyConstPtr &joy);
        void joyCallback(const sensor_msgs::JoyConstPtr &joy);

    };

    double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis);
    bool getButton(const sensor_msgs::JoyConstPtr &joy, const int button);

}

#endif
