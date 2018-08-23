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

<<<<<<< HEAD
        private_nh_.param<int>("a_btn", buttons.a, 1);
        private_nh_.param<int>("b_btn", buttons.b, 2);
        private_nh_.param<int>("x_btn", buttons.x, 3);
        private_nh_.param<int>("y_btn", buttons.y, 4);

        private_nh_.param<double>("initial_z_cmd", zCommand, 0);
        private_nh_.param<double>("min_z_cmd", zCommandMin, -10.0);
        private_nh_.param<double>("max_z_cmd", zCommandMax, 10.0);
        private_nh_.param<double>("z_scale", zScale, 0.1);

        private_nh_.param<bool>("invert_x", invertX, false);

=======
        private_nh_.param<double>("initial_z_cmd", zCommand, 0);
        private_nh_.param<double>("min_z_command", zCommandMin, -10.0);
        private_nh_.param<double>("max_z_command", zCommandMax, 10.0);
        private_nh_.param<double>("z_scale", zScale, 0.1);

>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2
        //Control mode parameter
        std::string control_mode;
        private_nh_.param<std::string>("control_mode", control_mode, "attitude_altitude");

        if (control_mode == "attitude_altitude") {
            ROS_INFO_STREAM("Attitude + Altitude Hold Mode Enabled");
            
            //Limit parameters
            private_nh_.param<double>("yawrate_max", axes.yaw.max, 0.5);
<<<<<<< HEAD
            private_nh_.param<double>("pitch_max", axes.x.max, 0.15);
            private_nh_.param<double>("roll_max", axes.y.max, 0.15);
=======
            private_nh_.param<double>("pitch_max", axes.x.max, 20.0);
            private_nh_.param<double>("roll_max", axes.y.max, 20.0);
>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2

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
<<<<<<< HEAD
        } else if (control_mode == "x_btn_attitude_altitude") {
            ROS_INFO_STREAM("X-button Attitude + Altitude Control Mode Enabled");

            private_nh_.param<double>("pitch_max", axes.x.max, 0.15);
            joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyXBtnAttAltCallback, this, _1));
            attitude_publisher_ = node_handle_.advertise<AttitudeCommand>("/teleop_command/attitude", 1);
            altitude_publisher_ = node_handle_.advertise<AltitudeCommand>("/teleop_command/altitude", 1, true);

            //Set zmax (speed at which z position changes)
            axes.z.max = 0.05;
        } else if (control_mode == "dist_accel_altitude") {
            ROS_INFO_STREAM("Distributed Accel Attitude Control Mode Enabled");

            rcSwitchFlipped = false;
            private_nh_.param<int>("rc_channel", rcChannel, 6);
            rc_subscriber_ = node_handle_.subscribe<rosflight_msgs::RCRaw>("rc_raw", 1, boost::bind(&Teleop::rcRawCallback, this, _1));

            attitude_publisher_ = node_handle_.advertise<AttitudeCommand>("/teleop_command/attitude", 1);
            altitude_publisher_ = node_handle_.advertise<AltitudeCommand>("/teleop_command/altitude", 1, true);
=======
>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2
        } else {
            ROS_ERROR_STREAM("Unsupported control mode: " << control_mode);
        }

        ROS_INFO_STREAM("Teleop: Setting initial z to " << zCommand);

<<<<<<< HEAD
        if (control_mode.find("altitude") != std::string::npos) {
            //Publish initial altitude
            AltitudeCommand altCmd;
            altCmd.header.stamp = ros::Time::now();
            altCmd.z = zCommand;
            altitude_publisher_.publish(altCmd);
        }
    }

    void Teleop::rcRawCallback(const rosflight_msgs::RCRawConstPtr &msg) {
        if (rcSwitchFlipped && msg->values[rcChannel] <= 1500)
        {
            rcSwitchFlipped = false;
            ROS_WARN_STREAM("Attitude command stream deactivated");
        }
        else if (!rcSwitchFlipped && msg->values[rcChannel] > 1500)
        {
            rcSwitchFlipped = true;
            attCmdStart = ros::Time::now();
            ROS_WARN_STREAM("Attitude command stream activated");
        }

        AttitudeCommand attCmd;
        AltitudeCommand altCmd;
        attCmd.header.stamp = altCmd.header.stamp = ros::Time::now();
        attCmd.pitch = 0;
        attCmd.yaw_rate = 0;
        if (rcSwitchFlipped) {
            double secsSinceStart = (attCmd.header.stamp - attCmdStart).toSec();
            double multiplier = 1.0;

            if (secsSinceStart < 1.5) {
                multiplier = 1.0;
            } else if (secsSinceStart < 3.5) {
                multiplier = -1.0;
            } else if (secsSinceStart < 5.0) {
                multiplier = 1.0;
            } else if (secsSinceStart < 6.5) {
                multiplier = -1.0;
            } else if (secsSinceStart < 8.0) {
                multiplier = 1.0;
            } else if (secsSinceStart < 10.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 12.0) {
                multiplier = 1.0;
            } else if (secsSinceStart < 13.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 14.0) {
                multiplier = 1.0;
            } else if (secsSinceStart < 15.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 16.0) {
                multiplier = 1.0;
            } else if (secsSinceStart < 17.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 17.5) {
                multiplier = 1.0;
            } else if (secsSinceStart < 18.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 18.5) {
                multiplier = 1.0;
            } else if (secsSinceStart < 19.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 19.25) {
                multiplier = 1.0;
            } else if (secsSinceStart < 19.5) {
                multiplier = -1.0;
            } else if (secsSinceStart < 19.75) {
                multiplier = 1.0;
            } else if (secsSinceStart < 20.0) {
                multiplier = -1.0;
            } else if (secsSinceStart < 20.125) {
                multiplier = 1.0;
            } else if (secsSinceStart < 20.25) {
                multiplier = -1.0;
            } else if (secsSinceStart < 20.375) {
                multiplier = 1.0;
            } else if (secsSinceStart < 20.5) {
                multiplier = -1.0;
            } else if (secsSinceStart < 20.6) {
                multiplier = 1.0;
            } else if (secsSinceStart < 20.7) {
                multiplier = -1.0;
            } else if (secsSinceStart < 20.8) {
                multiplier = 1.0;
            } else if (secsSinceStart < 20.9) {
                multiplier = -1.0;
            }

            else {
                multiplier = 0.0;
                ROS_WARN_ONCE("Attitude command stream completed");
            }
            
            attCmd.roll = 0.3 * multiplier;

            //ROS_ERROR_STREAM(secsSinceStart << ": " << attCmd.roll);
        } else {
            attCmd.roll = 0;
        }
        attitude_publisher_.publish(attCmd);

=======
        //Publish initial altitude
        AltitudeCommand altCmd;
        altCmd.header.stamp = ros::Time::now();
>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2
        altCmd.z = zCommand;
        altitude_publisher_.publish(altCmd);
    }

    void Teleop::joyAttAltCallback(const sensor_msgs::JoyConstPtr &joy) {
        AttitudeCommand attCmd;
        AltitudeCommand altCmd;

        //Populate and publish attitude command
        attCmd.header.stamp = altCmd.header.stamp = ros::Time::now();
<<<<<<< HEAD
        attCmd.roll = getAxis(joy, axes.x);
        if (!invertX) {
            attCmd.roll = -1.0*attCmd.roll;
        }
        attCmd.pitch = -1.0*getAxis(joy, axes.y);
        attCmd.yaw_rate = getAxis(joy, axes.yaw);
        if (!invertX) {
            attCmd.yaw_rate = -1.0*attCmd.yaw_rate;
        }
=======
        attCmd.roll = -1.0*getAxis(joy, axes.x);
        attCmd.pitch = -1.0*getAxis(joy, axes.y);
        attCmd.yaw_rate = -1.0*getAxis(joy, axes.yaw);
>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2
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

<<<<<<< HEAD
    void Teleop::joyXBtnAttAltCallback(const sensor_msgs::JoyConstPtr &joy) {
        AttitudeCommand attCmd;
        AltitudeCommand altCmd;

        //Populate and publish attitude command
        attCmd.header.stamp = altCmd.header.stamp = ros::Time::now();
        attCmd.roll = 0;
        if (getButton(joy, buttons.a)) {
            attCmd.pitch = -1.0*fabs(axes.x.max);
        } else {
            attCmd.pitch = 0;
        }
        attCmd.yaw_rate = getAxis(joy, axes.yaw);
        if (!invertX) {
            attCmd.yaw_rate = -1.0*attCmd.yaw_rate;
        }
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

=======
>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2
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
<<<<<<< HEAD

    bool getButton(const sensor_msgs::JoyConstPtr &joy, const int button) {
        if (button <= 0 || button > joy->buttons.size()) {
            ROS_ERROR_STREAM("Button " << button << " out of range, joy has " << joy->buttons.size() << " buttons");
            return false;
        }

        return joy->buttons[button - 1] > 0;
    }
=======
>>>>>>> d6e757694bff8059abeeb42c9ff1fb61b70d48a2
}
