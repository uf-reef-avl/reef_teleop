#ifndef REEF_TELEOP_TELEOP_H
#define REEF_TELEOP_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <rosflight_msgs/RCRaw.h>

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
        ros::Subscriber joy_subscriber_, rc_subscriber_;
        ros::Publisher attitude_publisher_, altitude_publisher_, velocity_publisher_;

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
        ros::Time attCmdStart;

    public:
        Teleop();
        void rcRawCallback(const rosflight_msgs::RCRawConstPtr &msg);
        void joyAttAltCallback(const sensor_msgs::JoyConstPtr &joy);
        void joyVelAltCallback(const sensor_msgs::JoyConstPtr &joy);
        void joyXBtnAttAltCallback(const sensor_msgs::JoyConstPtr &joy);
    };

    double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis);
    bool getButton(const sensor_msgs::JoyConstPtr &joy, const int button);

}

#endif
