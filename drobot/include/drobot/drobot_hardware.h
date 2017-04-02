#ifndef DROBOT_HARDWARE_H
#define DROBOT_HARDWARE_H

#include "ros/ros.h"
#include <string>
#include "drobot/legacy_wrapper/drobot.h"
#include "drobot/legacy_wrapper.h"
#include "drobot_msgs/DrobotControl.h"
#include "drobot_msgs/DrobotOdom.h"
#include "drobot_msgs/DrobotStatus.h"

namespace drobot_base
{
	class DrobotHardware
	{
	public:
		DrobotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

		void writeCommandsToHardware();

		void updateHardware();

		void updateDiagnostics();

		void command_callback(const drobot_msgs::DrobotControl &control_msg);

	private:
		void limitAngleSpeed(drobot_msgs::DrobotControl &control_msg);
		ros::NodeHandle nh_, private_nh_;

		ros::Publisher pub_odometry;

		ros::Publisher pub_diagnostic;

		ros::Subscriber sub_control; 

		drobot_msgs::DrobotControl cmd_msgs;

		drobot_msgs::DrobotOdom odom_msgs;

		drobot_msgs::DrobotStatus status_msgs;

		// ROS Parameters
		double wheel_diameter_, max_speed_;

		double left_front_angle_offset_, right_front_angle_offset_;
		double left_rear_angle_offset_, right_rear_angle_offset_;

		double left_front_steer_max_, left_front_steer_min_;
		double right_front_steer_max_, right_front_steer_min_;
		double left_rear_steer_max_, left_rear_steer_min_;
		double right_rear_steer_max_, right_rear_steer_min_;

		double polling_timeout_;
	};

} //namespace drobot_base

#endif  // DROBOT_HARDWARE_H
