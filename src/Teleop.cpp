#include <reef_teleop/AttitudeCommand.h>
#include <reef_teleop/AltitudeCommand.h>

#include "Teleop.h"

namespace reef_teleop {

    Teleop::Teleop() : private_nh_("~") {
        //Controller layout parameters - defaults to xbox layout
        private_nh_.param<int>("x_axis", axes.x.axis, 4);
        private_nh_.param<int>("y_axis", axes.y.axis, 5);
        private_nh_.param<int>("z_axis", axes.z.axis, 2);
        private_nh_.param<int>("thrust_axis", axes.thrust.axis, 2);
        private_nh_.param<int>("yaw_axis", axes.yaw.axis, 1);

        //Limit parameters
        private_nh_.param<double>("yaw_velocity_max", axes.yaw.max, 90.0);
        private_nh_.param<double>("pitch_max", axes.x.max, 20.0);
        private_nh_.param<double>("roll_max", axes.y.max, 20.0);

        private_nh_.param<double>("initial_z_cmd", zCommand, 0);
        private_nh_.param<double>("min_z_command", zCommandMin, -10.0);
        private_nh_.param<double>("max_z_command", zCommandMax, 10.0);

        //Control mode parameter
        std::string control_mode;
        private_nh_.param<std::string>("control_mode", control_mode, "attitude_altitude_hold");

        if (control_mode == "attitude_altitude_hold") {
            ROS_INFO_STREAM("Attitude + Altitude Hold Mode Enabled");
            joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyAttAltCallback, this, _1));
            
            attitude_publisher_ = node_handle_.advertise<AttitudeCommand>("/teleop_command/attitude", 10);
            altitude_publisher_ = node_handle_.advertise<AltitudeCommand>("/teleop_command/altitude", 10);

            //Set zmax (speed at which position changes)
            axes.z.max = 0.05;
        } else {
            ROS_ERROR_STREAM("Unsupported control mode: " << control_mode);
        }
    }

    void Teleop::joyAttAltCallback(const sensor_msgs::JoyConstPtr &joy) {
        AttitudeCommand attCmd;
        AltitudeCommand altCmd;

        //Populate and publish attitude command
        attCmd.header.stamp = altCmd.header.stamp = ros::Time::now();
        attCmd.roll = -1.0*getAxis(joy, axes.x);
        attCmd.pitch = -1.0*getAxis(joy, axes.y);
        attCmd.yaw_rate = -1.0*getAxis(joy, axes.yaw);
        attitude_publisher_.publish(attCmd);

        //Populate and publish altitude command
        zCommand -= getAxis(joy, axes.z);
        if (zCommand < zCommandMin) {
            zCommand = zCommandMin;
        } else if (zCommand > zCommandMax) {
            zCommand = zCommandMax;
        }
        altCmd.z = zCommand;
        altitude_publisher_.publish(altCmd);
    }

    double getAxis(const sensor_msgs::JoyConstPtr &joy, const Axis &axis) {
        //Axis out of range
        if (axis.axis == 0 || std::abs(axis.axis) > joy->axes.size()) {
            ROS_ERROR_STREAM("Axis " << axis.axis << " out of range, joy has " << joy->axes.size() << " axes");
            return 0;
        }

        double output = std::abs(axis.axis) / axis.axis * joy->axes[std::abs(axis.axis) - 1] * axis.max + axis.offset;

        //5% stick deadzone
        if (std::abs(output) < axis.max * 0.05) {
            output = 0.0;
        }

        return output;
    }
}