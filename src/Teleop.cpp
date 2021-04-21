#include "Teleop.h"

namespace reef_teleop {

Teleop::Teleop() : private_nh_("~"), button_pressed(false), lost_aruco(false),X_TOLERANCE(0.1),
Y_TOLERANCE(0.1),ENGAGE_DESCENT(0){
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
    private_nh_.param<double>("aruco_threshold", aruco_threshold, 0.5);
    private_nh_.param<bool>("invert_x", invertX, false);

    last_aruco_time = ros::Time::now();

    private_nh_.param<double>("yawrate_max", axes.yaw.max, 0.5);
    //Set zmax (speed at which position changes)
    axes.z.max = 0.05;

    joy_subscriber_ =
        node_handle_.subscribe<sensor_msgs::Joy>("joy", 10, boost::bind(&Teleop::joyCallback, this, _1));
    aruco_subscriber =
        node_handle_.subscribe<geometry_msgs::PoseStamped>("aruco_single/pose", 10, &Teleop::aruco_callback, this);

    desired_state_publisher_ = node_handle_.advertise<reef_msgs::DesiredState>("desired_state", 1);

    imu_subscriber_ = node_handle_.subscribe<sensor_msgs::Imu>("imu/data", 1, &Teleop::imuCallback, this);

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
    landing_msg.pose.z = zCommand;
}


void Teleop::joyCallback(const sensor_msgs::JoyConstPtr &joy) {


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
    else if (control_mode == "altitude_hold")
        desired_state_msg.altitude_only = true;

    if(button_pressed && (joy->header.stamp - last_aruco_time).toSec() > aruco_threshold) {
        lost_aruco = true;
        ROS_WARN_STREAM("I had aruco but I lost it!");
    } else
        lost_aruco =false;

    if(getButton(joy, buttons.a)) {
        button_pressed = true;
    }

    if(!button_pressed || lost_aruco)
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

void Teleop::aruco_callback(const geometry_msgs::PoseStampedConstPtr& msg){

    ROS_WARN_STREAM("Got Aruco");

    if(!button_pressed)
        return;

    last_aruco_time = msg->header.stamp;
    geometry_msgs::PoseStamped pose_board_in_camera_frame;
    pose_board_in_camera_frame = *msg;
    Eigen::Matrix3d C_target_to_camera;
    Eigen::Matrix3d C_body_to_camera;
    Eigen::Matrix3d C_body_to_body_level;
    if(!ENGAGE_DESCENT && pose_board_in_camera_frame.pose.position.x <= X_TOLERANCE && pose_board_in_camera_frame.pose.position.y <= Y_TOLERANCE){
        ENGAGE_DESCENT = 1;
    }

    //Construct all matrices next.
    Eigen::Quaterniond quat_target_to_camera;
    quat_target_to_camera.x() = pose_board_in_camera_frame.pose.orientation.x;
    quat_target_to_camera.y() = pose_board_in_camera_frame.pose.orientation.y;
    quat_target_to_camera.z() = pose_board_in_camera_frame.pose.orientation.z;
    quat_target_to_camera.w() = pose_board_in_camera_frame.pose.orientation.w;

    C_target_to_camera = reef_msgs::quaternion_to_rotation(quat_target_to_camera);
    C_body_to_camera<< 0, 1 , 0,
                       -1, 0,  0,
                       0, 0,  1;
    Eigen::Matrix3d C1;
    C1 << 1,        0 ,             0,
         0,     cos(inertial_to_body_roll),  sin(inertial_to_body_roll),
         0,     -sin(inertial_to_body_roll),  cos(inertial_to_body_roll);

    Eigen::Matrix3d C2;
    C2 << cos(inertial_to_body_pitch),  0, -sin(inertial_to_body_pitch),
         0,                             1,           0 ,
         sin(inertial_to_body_pitch),    0,  cos(inertial_to_body_pitch);

    C_body_to_body_level <<   C2.transpose()*C1.transpose();

    //Compute the velocity command. This control law wants to reduce the relative position vector
    // of the quadcopter with respect to the board.
    //The position (Z)  and (XY) velocity commands are computed in the body-level frame.

    //The auto-landing algorithm first aligns the XY coordinates of the quad with those of the charuco target.
    //The diagonal matrix, K_mode, has three elements that need to be either 0 or 1. The element at (1,1) and (2,2)
    // enable the autolanding algorithm to take over XY, whereas the (3,3) element enables the autolanding to control
    //altitude.
    // To start descent, switch element (3,3) to one.
    Eigen::Matrix3d K_mode;
    K_mode << 1,0,0,
              0,1,0,
              0,0,ENGAGE_DESCENT;

    Eigen::Vector3d command_vector;

    //command_vector contains:
    // the X velocity in command_vector(0)
    //the Y velocity in command_vector(1)
    //the Z increment in command_vector(2)
    Eigen::Matrix3d K_gains;
    K_gains << 0.12,0,0,
               0,0.12,0,
               0,0,0.035;
    Eigen::Vector3d aruco_position_in_camera_frame;
    aruco_position_in_camera_frame<< pose_board_in_camera_frame.pose.position.x,pose_board_in_camera_frame.pose.position.y,pose_board_in_camera_frame.pose.position.z;
    //Command vector is the result from our control law.
    command_vector<< -K_gains * C_body_to_camera.transpose()*C_target_to_camera*K_mode*C_target_to_camera.transpose()*(-aruco_position_in_camera_frame);
    landing_msg.velocity_valid = true;
    if(ENGAGE_DESCENT)
        zCommand += command_vector(2);
//    else
    landing_msg.pose.z = zCommand;
    landing_msg.velocity.x = command_vector(0);
    landing_msg.velocity.y = command_vector(1);
    desired_state_publisher_.publish(landing_msg);
    if(-landing_msg.pose.z <= 0.10){
        joy_subscriber_.shutdown();
        landing_msg.pose.z = 0;
        landing_msg.velocity.x = 0;
        landing_msg.velocity.y = 0;
        for(int i =0; i< 1000; i++)
            desired_state_publisher_.publish(landing_msg);
        aruco_subscriber.shutdown();
    }

}

 void Teleop::imuCallback(const sensor_msgs::ImuConstPtr &msg_imu){
     sensor_msgs::Imu msg;
     msg = *msg_imu;
     Eigen::Quaterniond q_inertial_to_body;
     q_inertial_to_body.x() = msg.orientation.x;
     q_inertial_to_body.y() =  msg.orientation.y;
     q_inertial_to_body.z() =  msg.orientation.z;
     q_inertial_to_body.w() =  msg.orientation.w;

     Eigen::Matrix3d C_inertial_to_body;
     C_inertial_to_body = q_inertial_to_body.toRotationMatrix().transpose();
     reef_msgs::roll_pitch_yaw_from_rotation321(C_inertial_to_body, inertial_to_body_roll, inertial_to_body_pitch, inertial_to_body_yaw);
}

}
