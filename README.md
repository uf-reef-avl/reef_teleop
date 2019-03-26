# REEF Teleop
The REEF Teleop package provides ROS [joy](http://wiki.ros.org/joy) controller remappings designed specifically for teleoperation of the [REEF Control](http://192.168.1.101/AVL-Summer-18/reef_control) package. It was modeled after the [hector_quadrotor_teleop](http://wiki.ros.org/hector_quadrotor_teleop) package.

## Prerequisites
Requires [ROS Kinetic](http://wiki.ros.org/kinetic/Installation) to be installed along with the [joy](http://wiki.ros.org/joy) package.

## Installation
Simply clone **reef_teleop** to the catkin workspace src directory and compile it to make sure everything works.
```
sudo apt-get install ros-<distro>-joy
cd catkin_ws/src
git clone http://192.168.1.101/AVL-Summer-18/reef_teleop
cd ../ && catkin_make
```
## Usage
REEF Teleop is designed to allow the construction of launchfile profiles for different teleop input devices, such as Logitech gamepads or Xbox controllers. As long as the device is compatible with joy, it is compatible with REEF Teleop. Before constructing a launchfile profile for a new device, familiarize oneself with the parameters in the following section.

### Parameters

|Name|Type|Description|Default|
|--|--|--|--|
|**x_axis**|integer|Joy axis mapping to x control axis|4|
|**y_axis**|integer|Joy axis mapping to y control axis|5|
|**z_axis**|integer|Joy axis mapping to z control axis|2|
|**thrust_axis**|integer|Joy axis mapping to thrust axis|2|
|**yaw_axis**|integer|Joy axis mapping to yaw control axis|1|
|**a_btn**|integer|Joy button mapping to controller A button|1|
|**b_btn**|integer|Joy button mapping to controller B button|2|
|**x_btn**|integer|Joy button mapping to controller X button|3|
|**y_btn**|integer|Joy button mapping to controller Y button|4|
|**initial_z_cmd**|double|Initial altitude command (meters) |0|
|**min_z_cmd**|double|Minimum altitude command (meters)|-10.0|
|**max_z_cmd**|double|Maximum altitude command (meters)|10.0|
|**z_scale**|double|Multiplier for altitude input commands|0.1|
|**invert_x**|bool|Inverts controller x commands|false|
|**control_mode**|std::string|Teleoperation control mode|attitude_altitude|
|**pitch_max**|double|Maximum pitch command output (radians)|0.15|
|**roll_max**|double|Maximum roll command output (radians)|0.15|
|**yawrate_max**|double|Maximum yawrate command output (rad/s)|0.5|
|**x_dot_max**|double|Maximum x velocity command output (m/s)|0.25|
|**y_dot_max**|double|Maximum y velocity command output (m/s)|0.25|

### Control Modes

Currently, REEF Teleop supports the follwoing control modes:

 - **Attitude + Altitude**
Publishes pitch and roll attitude, yawrate, and altitude setpoints. A change on the controller z axis corresponds to a proportional change in the altitude setpoint governed by the **z_scale** multiplier parameter. To enable this mode, set the **control_mode** parameter to "attitude_altitude".

 - **Velocity + Altitude**
Publishes x and y velocity, yawrate, and altitude setpoints. Altitude functions the same as with Attitude + Altitude mode. To enable this mode, set the **control_mode** parameter to "velocity_altitude".

 - **Altitude Only**
Publishes only the altitude setpoint. To enable this mode, set the **control_mode** parameter to "altitude_only".

### ROS Topics and Messages

#### Subscribed Topics
|Topic Name|Message Type|Description|
|--|--|--|
|joy|sensor_msgs::Joy|Joy controller data stream|

#### Published Topics
|Topic Name|Message Type|Description|
|--|--|--|
|desired_state|reef_msgs::DesiredState|Contains desired state|


### Launchfile Examples
Example launchfile for XBox One bluetooth controller:
```xml
<?xml version="1.0"?>

<launch>
    <arg name="joy_dev" default="/dev/input/js0" />

    <node name="joy" pkg="joy" type="joy_node" output="screen" >
        <param name="dev" value="$(arg joy_dev)" />
        <param name="autorepeat_rate" value="100" />
        <param name="coalesce_interval" value="0.01" />
    </node>

    <arg name="control_mode" default="attitude_altitude_hold"/>

    <arg name="thrust_max" default="0.25"/>
    <arg name="pitch_max" default="0.25"/>
    <arg name="roll_max" default="0.25"/>
    <arg name="yawrate_max" default="0.25"/>

    <arg name="x_dot_max" default="0.25"/>
    <arg name="y_dot_max" default="0.25"/>

    <arg name="initial_z_cmd" default="0"/>
    <arg name="min_z_cmd" default="-10.0"/>
    <arg name="max_z_cmd" default="10.0"/>
    <arg name="z_scale" default="0.1"/>

    <node name="reef_teleop" pkg="reef_teleop" type="reef_teleop" output="screen">
        <rosparam subst_value="true">
            control_mode: $(arg control_mode)

            x_axis: 3
            y_axis: 4
            z_axis: 2
            thrust_axis: 2
            yaw_axis: 1

            initial_z_cmd: $(arg initial_z_cmd)
            min_z_cmd: $(arg min_z_cmd)
            max_z_cmd: $(arg max_z_cmd)
            z_scale: $(arg z_scale)

            pitch_max: $(arg pitch_max)
            roll_max: $(arg roll_max)
            thrust_max: $(arg thrust_max)
            yawrate_max: $(arg yawrate_max)

            x_dot_max: $(arg x_dot_max)
            y_dot_max: $(arg y_dot_max)
        </rosparam>
    </node>
</launch>
```
Example launchfile for Logitech wireless USB controller:
```xml
<?xml version="1.0"?>

<launch>
    <arg name="joy_dev" default="/dev/input/js0" />

    <node name="joy" pkg="joy" type="joy_node" output="screen" >
        <param name="dev" value="$(arg joy_dev)" />
        <param name="autorepeat_rate" value="100" />
        <param name="coalesce_interval" value="0.01" />
    </node>

    <arg name="control_mode" default="attitude_altitude"/>

    <arg name="thrust_max" default="0.25"/>
    <arg name="pitch_max" default="0.25"/>
    <arg name="roll_max" default="0.25"/>
    <arg name="yawrate_max" default="0.25"/>

    <arg name="x_dot_max" default="0.25"/>
    <arg name="y_dot_max" default="0.25"/>

    <arg name="initial_z_cmd" default="0"/>
    <arg name="min_z_cmd" default="-10.0"/>
    <arg name="max_z_cmd" default="10.0"/>
    <arg name="z_scale" default="0.1"/>

    <arg name="invert_x" default="false"/>

    <node name="reef_teleop" pkg="reef_teleop" type="reef_teleop" output="screen">
        <rosparam subst_value="true">
            control_mode: $(arg control_mode)

            invert_x: $(arg invert_x)
            x_axis: 4
            y_axis: 5
            z_axis: 2
            thrust_axis: 2
            yaw_axis: 1

            initial_z_cmd: $(arg initial_z_cmd)
            min_z_cmd: $(arg min_z_cmd)
            max_z_cmd: $(arg max_z_cmd)
            z_scale: $(arg z_scale)

            pitch_max: $(arg pitch_max)
            roll_max: $(arg roll_max)
            thrust_max: $(arg thrust_max)
            yawrate_max: $(arg yawrate_max)

            x_dot_max: $(arg x_dot_max)
            y_dot_max: $(arg y_dot_max)
        </rosparam>
    </node>
</launch>

```