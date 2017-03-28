#ifndef TELEOP_TWIST_H
#define TELEOP_TWIST_H

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float64MultiArray.h"
#include "drobot_msgs/DrobotControl.h"
#include "drobot_control/control_convert.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class teleop_twist
{
public:
	teleop_twist();
	
	void teleopLoop();

	void joyStick_callback(const sensor_msgs::Joy &msg);//摇杆控制

	drobot_msgs::DrobotControl control_msgs;

private:
	float js_axes[8];
  	int js_buttons[11];

  	bool stopFlag;

  	ros::NodeHandle nh;
  	ros::Publisher cmd_pub;
  	ros::Subscriber sub_js; //joy stick sub
};

#endif