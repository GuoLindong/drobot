#include "drobot/drobot_hardware.h"
#include "ros/callback_queue.h"
#include "drobot_msgs/DrobotControl.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

void controlLoop(drobot_base::DrobotHardware &drobot)
{
	drobot.updateHardware();
	drobot.writeCommandsToHardware();
}

void diagnosticLoop(drobot_base::DrobotHardware &drobot)
{
  drobot.updateDiagnostics();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "drobot_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  drobot_msgs::DrobotControl cmd_msgs;

  drobot_base::DrobotHardware drobot(nh, private_nh, control_frequency);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue drobot_queue;
  ros::AsyncSpinner drobot_spinner(1, &drobot_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(ros::Duration(1 / control_frequency), boost::bind(controlLoop, boost::ref(drobot)), &drobot_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions diagnostic_timer(ros::Duration(1 / diagnostic_frequency), boost::bind(diagnosticLoop, boost::ref(drobot)), &drobot_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);


  drobot_spinner.start();
  
  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
