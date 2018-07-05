#include <reef_teleop/AttitudeCommand.h>
#include <reef_teleop/AltitudeCommand.h>
#include <reef_teleop/VelocityCommand.h>

#include "Teleop.h"

namespace reef_teleop {

    Teleop::Teleop() : private_nh_("~") {
        //Controller layout parameters - defaults to xbox layout
        private_nh_.param<int>("x_axis", axes.x.axis, 4);
        private_nh_.param<int>("y_axis", axes.y.axis, 5);
        private_nh_.param<int>("z_axis", axes.z.axis, 2);
        private_nh_.param<int>("thrust_axis", axes.thrust.axis, 2);
        private_nh_.param<int>("yaw_axis", axes.yaw.axis, 1);

        private_nh_.param<double>("initial_z_cmd", zCommand, 0);
        private_nh_.param<double>("min_z_command", zCommandMin, -10.0);
        private_nh_.param<double>("max_z_command", zCommandMax, 10.0);
        private_nh_.param<double>("z_scale", zScale, 0.1);

        //Control mode parameter
        std::string control_mode;
        private_nh_.param<std::string>("control_mode", control_mode, "attitude_altitude");

        if (control_mode == "attitude_altitude") {
            ROS_INFO_STREAM("Attitude + Altitude Hold Mode Enabled");
            
            //Limit parameters
            private_nh_.param<double>("yawrate_max", axes.yaw.max, 0.5);
            private_nh_.param<double>("pitch_max", axes.x.max, 20.0);
            private_nh_.param<double>("roll_max", axes.y.max, 20.0);

            joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyAttAltCallback, this, _1));
            attitude_publisher_ = node_handle_.advertise<AttitudeCommand>("/teleop_command/attitude", 1);
            altitude_publisher_ = node_handle_.advertise<AltitudeCommand>("/teleop_command/altitude", 1, true);

            //Set zmax (speed at which position changes)
            axes.z.max = 0.05;
        } else if (control_mode == "velocity_altitude") {
            ROS_INFO_STREAM("Velocity + Altitude Control Mode Enabled");

            //Limit parameters
            private_nh_.param<double>("x_dot_max", axes.x.max, 0.25);
            private_nh_.param<double>("y_dot_max", axes.y.max, 0.25);
            private_nh_.param<double>("yawrate_max", axes.yaw.max, 0.5);

            joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyVelAltCallback, this, _1));
            velocity_publisher_ = node_handle_.advertise<VelocityCommand>("/teleop_command/velocity", 1);
            altitude_publisher_ = node_handle_.advertise<AltitudeCommand>("/teleop_command/altitude", 1, true);

            //Set zmax (speed at which position changes)
            axes.z.max = 0.05;
        } else {
            ROS_ERROR_STREAM("Unsupported control mode: " << control_mode);
        }

        ROS_INFO_STREAM("Teleop: Setting initial z to " << zCommand);

        //Publish initial altitude
        AltitudeCommand altCmd;
        altCmd.header.stamp = ros::Time::now();
        altCmd.z = zCommand;
        altitude_publisher_.publish(altCmd);
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
        zCommand -= zScale*getAxis(joy, axes.z);
        if (zCommand < zCommandMin) {
            zCommand = zCommandMin;
        } else if (zCommand > zCommandMax) {
            zCommand = zCommandMax;
        }
        altCmd.z = zCommand;
        altitude_publisher_.publish(altCmd);
    }

    void Teleop::joyVelAltCallback(const sensor_msgs::JoyConstPtr &joy) {
        VelocityCommand velCmd;
        AltitudeCommand altCmd;

        //Populate and publish attitude command
        velCmd.header.stamp = altCmd.header.stamp = ros::Time::now();
        velCmd.y_dot = -1.0*getAxis(joy, axes.x);
        velCmd.x_dot = getAxis(joy, axes.y);
        velCmd.yaw_rate = -1.0*getAxis(joy, axes.yaw);
        velocity_publisher_.publish(velCmd);

        //Populate and publish altitude command
        zCommand -= zScale*getAxis(joy, axes.z);
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
