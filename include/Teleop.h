#ifndef REEF_TELEOP_TELEOP_H
#define REEF_TELEOP_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <rosflight_msgs/RCRaw.h>
#include <reef_msgs/DesiredState.h>
#include <reef_msgs/dynamics.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>

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
        ros::Subscriber  aruco_subscriber;
        ros::Subscriber imu_subscriber_;

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
        reef_msgs::DesiredState desired_state_msg;
        reef_msgs::DesiredState landing_msg;
        bool invertX;
        bool rcSwitchFlipped;
        int rcChannel;
        std::string control_mode;
        bool button_pressed;
        int land_idx;
        double aruco_threshold;
        ros::Time last_aruco_time;
        bool lost_aruco;
        int ENGAGE_DESCENT;
        double X_TOLERANCE;
        double Y_TOLERANCE;
        double inertial_to_body_roll;
        double inertial_to_body_pitch;
        double inertial_to_body_yaw;

    public:
        Teleop();
        void joyAttAltCallback(const sensor_msgs::JoyConstPtr &joy);
        void joyCallback(const sensor_msgs::JoyConstPtr &joy);
        void aruco_callback(const geometry_msgs::PoseStampedConstPtr& msg);
        void imuCallback(const sensor_msgs::ImuConstPtr &msg);


        };

    double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis);
    bool getButton(const sensor_msgs::JoyConstPtr &joy, const int button);

}

#endif
