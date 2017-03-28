#include "drobot/legacy_wrapper/drobot.h"
#include <ros/ros.h>
#include "drobot_msgs/DrobotControl.h"

namespace
{
  std::string port_;
}

namespace legacy_wrapper
{
	  void reconnect()
    {
    	if (port_.empty())
    	{
      		throw std::logic_error("Can't reconnect when port is not configured");
    	}
    	ROS_INFO_STREAM("Connecting to Drobot on port " << port_ << "...");
    	drobot::Transport::instance().configure(port_.c_str(), 3);
    	ROS_INFO("Connected");
  	}

  	void connect(std::string port)
    {
    	port_ = port;
   	 	reconnect();
  	}

  	void configureLimits(double left_front_angle_offset, double right_front_angle_offset, double left_rear_angle_offset, double right_rear_angle_offset)
  	{
  	  bool success = false;
  	  while (!success)
  	  {
  	    try
  	    {
  	      drobot::SetAngleOffset(left_front_angle_offset, right_front_angle_offset, left_rear_angle_offset, right_rear_angle_offset).send();

  	      success = true;
  	    }
  	    catch (drobot::Exception *ex)
  	    {
  	      ROS_ERROR_STREAM("Error configuring angle offset: " << ex->message);
  	      reconnect();
  	    }
  	  }
  	}

  	void controlSpeed(drobot_msgs::DrobotControl &msg)
  	{
  	  bool success = false;
  	  while (!success)
  	  {
  	    try
  	    {
  	      drobot::SetDifferentialControl(msg.left_front_speed, msg.left_front_steer, msg.right_front_speed, msg.right_front_steer, msg.left_rear_speed, msg.left_rear_steer, msg.right_rear_speed, msg.right_rear_steer).send();
  	      success = true;
  	    }
  	    catch (drobot::Exception *ex)
  	    {
  	      ROS_ERROR_STREAM("Error sending speed and accel command: " << ex->message);
  	      reconnect();
  	    }
  	  }
  	}
}