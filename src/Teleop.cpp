#include "Teleop.h"

namespace reef_teleop {

Teleop::Teleop() : private_nh_("~") {
    //Controller layout parameters - defaults to xbox layout
    private_nh_.param<int>("x_axis", axes.x.axis, 4);
    private_nh_.param<int>("y_axis", axes.y.axis, 5);
    private_nh_.param<int>("z_axis", axes.z.axis, 2);
    private_nh_.param<int>("thrust_axis", axes.thrust.axis, 2);
    private_nh_.param<int>("yaw_axis", axes.yaw.axis, 1);

    private_nh_.param<int>("a_btn", buttons.a, 1);
    private_nh_.param<int>("b_btn", buttons.b, 2);
    private_nh_.param<int>("x_btn", buttons.x, 3);
    private_nh_.param<int>("y_btn", buttons.y, 4);

    private_nh_.param<double>("initial_z_cmd", zCommand, 0);
    private_nh_.param<double>("min_z_cmd", zCommandMin, -10.0);
    private_nh_.param<double>("max_z_cmd", zCommandMax, 10.0);
    private_nh_.param<double>("z_scale", zScale, 0.1);
    private_nh_.param<bool>("invert_x", invertX, false);

    private_nh_.param<double>("yawrate_max", axes.yaw.max, 0.5);
    //Set zmax (speed at which position changes)
    axes.z.max = 0.05;

    joy_subscriber_ =
        node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyCallback, this, _1));

    desired_state_publisher_ = node_handle_.advertise<reef_msgs::DesiredState>("desired_state", 1);

    //Control mode parameter
    private_nh_.param<std::string>("control_mode", control_mode, "attitude_altitude");

    if (control_mode == "attitude_altitude") {
        ROS_INFO_STREAM("Attitude + Altitude Hold Mode Enabled");
        //Limit parameters
        private_nh_.param<double>("pitch_max", axes.x.max, 0.15);
        private_nh_.param<double>("roll_max", axes.y.max, 0.15);
    } else if (control_mode == "velocity_altitude") {
        ROS_INFO_STREAM("Velocity + Altitude Control Mode Enabled");
        //Limit parameters
        private_nh_.param<double>("x_dot_max", axes.x.max, 0.25);
        private_nh_.param<double>("y_dot_max", axes.y.max, 0.25);
    }
}


void Teleop::joyCallback(const sensor_msgs::JoyConstPtr &joy) {

    reef_msgs::DesiredState desired_state_msg;

    desired_state_msg.header.stamp = ros::Time::now();
    //Populate and publish altitude command
    zCommand -= zScale*getAxis(joy, axes.z);
    if (zCommand < zCommandMin) {
        zCommand = zCommandMin;
    } else if (zCommand > zCommandMax) {
        zCommand = zCommandMax;
    }
    desired_state_msg.pose.z = zCommand;

    if (control_mode == "attitude_altitude") {

        desired_state_msg.attitude_valid = true;
        desired_state_msg.attitude.x = getAxis(joy, axes.x);
        if (!invertX) {
            desired_state_msg.attitude.x = -1.0*desired_state_msg.attitude.x;
        }
        desired_state_msg.attitude.y = -1.0*getAxis(joy, axes.y);
        desired_state_msg.attitude.yaw = getAxis(joy, axes.yaw);
        if (!invertX) {
            desired_state_msg.attitude.yaw = -1.0*desired_state_msg.attitude.yaw;
        }

    }
    else if (control_mode == "velocity_altitude") {
        desired_state_msg.velocity_valid = true;
        desired_state_msg.velocity.x = getAxis(joy, axes.y);
        desired_state_msg.velocity.y = -1.0*getAxis(joy, axes.x);
        desired_state_msg.velocity.yaw = -1.0*getAxis(joy, axes.yaw);
    }
    desired_state_publisher_.publish(desired_state_msg);
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

bool getButton(const sensor_msgs::JoyConstPtr &joy, const int button) {
    if (button <= 0 || button > joy->buttons.size()) {
        ROS_ERROR_STREAM("Button " << button << " out of range, joy has " << joy->buttons.size() << " buttons");
        return false;
    }

    return joy->buttons[button - 1] > 0;
}

}
