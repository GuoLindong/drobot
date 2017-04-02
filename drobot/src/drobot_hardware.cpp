#include "drobot/drobot_hardware.h"
#include <boost/assign/list_of.hpp>
#include <cstdlib>
#include "drobot/legacy_wrapper/drobot.h"

namespace drobot_base
{

  /**
  * Initialize Drobot hardware
  */
  DrobotHardware::DrobotHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
    :
    nh_(nh),
    private_nh_(private_nh)
  {
    private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.254);
    private_nh_.param<double>("max_speed", max_speed_, 2.0);
    private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

    private_nh_.param<double>("left_front_angle_offset", left_front_angle_offset_, 0.0);
    private_nh_.param<double>("right_front_angle_offset", right_front_angle_offset_, 0.0);
    private_nh_.param<double>("left_rear_angle_offset", left_rear_angle_offset_, 0.0);
    private_nh_.param<double>("right_rear_angle_offset", right_rear_angle_offset_, 0.0);

    private_nh_.param<double>("left_front_steer_max_", left_front_steer_max_, 11.0); 
    private_nh_.param<double>("left_front_steer_min_", left_front_steer_min_, -51.0);
    private_nh_.param<double>("right_front_steer_max_", right_front_steer_max_, 51.0); 
    private_nh_.param<double>("right_front_steer_min_", right_front_steer_min_, -11.0); 
    private_nh_.param<double>("left_rear_steer_max_", left_rear_steer_max_, 51.0); 
    private_nh_.param<double>("left_rear_steer_min_", left_rear_steer_min_, -11.0); 
    private_nh_.param<double>("right_rear_steer_max_", right_rear_steer_max_, 11.0); 
    private_nh_.param<double>("right_rear_steer_min_", right_rear_steer_min_, -51.0);  

    std::string port;
    private_nh_.param<std::string>("port", port, "/dev/ttyUSB0");


    cmd_msgs.left_front_speed = 0; 
    cmd_msgs.right_front_speed = 0;
    cmd_msgs.left_rear_speed = 0;
    cmd_msgs.right_rear_speed = 0;

    cmd_msgs.left_front_steer = 0; 
    cmd_msgs.right_front_steer = 0;
    cmd_msgs.left_rear_steer = 0;
    cmd_msgs.right_rear_steer = 0;

    cmd_msgs.stop_mode = true;
    cmd_msgs.point_turn_mode = false;

    legacy_wrapper::connect(port);
    
    legacy_wrapper::configureLimits(left_front_angle_offset_, right_front_angle_offset_, left_rear_angle_offset_, right_rear_angle_offset_);

    sub_control = nh_.subscribe("command_info",1,&DrobotHardware::command_callback,this);

    pub_odometry = nh_.advertise<drobot_msgs::DrobotOdom>("odom_info", 10);

    pub_diagnostic = nh_.advertise<drobot_msgs::DrobotStatus>("status_info", 10);
  }

  void DrobotHardware::command_callback(const drobot_msgs::DrobotControl &control_msg)
  {
      cmd_msgs = control_msg;
  }

  /**
  * Get latest command info, and send to MCU
  */
  void DrobotHardware::writeCommandsToHardware()
  {
    limitAngleSpeed(cmd_msgs);

    legacy_wrapper::controlSpeed(cmd_msgs);
  }

  void DrobotHardware::updateDiagnostics()
  {
    legacy_wrapper::Channel<drobot::DataSystemStatus>::Ptr dia = legacy_wrapper::Channel<drobot::DataSystemStatus>::requestData(
      polling_timeout_);
    if(dia)
    {
      status_msgs.battery_voltage = dia->getVoltage();
      status_msgs.platform_temp = dia->getTemperature();

      status_msgs.lf_drive_pwm = dia->getLfDrivePwm();
      status_msgs.lf_turn_pwm = dia->getLfTurnPwm();
      status_msgs.lf_error = dia->getLfError();

      status_msgs.rf_drive_pwm = -dia->getRfDrivePwm();
      status_msgs.rf_turn_pwm = dia->getRfTurnPwm();
      status_msgs.rf_error = dia->getRfError();

      status_msgs.lr_drive_pwm = dia->getLrDrivePwm();
      status_msgs.lr_turn_pwm = dia->getLrTurnPwm();
      status_msgs.lr_error = dia->getLrError();

      status_msgs.rr_drive_pwm = -dia->getRrDrivePwm();
      status_msgs.rr_turn_pwm = dia->getRrTurnPwm();
      status_msgs.rr_error = dia->getRrError();
      pub_diagnostic.publish(status_msgs);
    }

  }

  void DrobotHardware::updateHardware()
  {

    legacy_wrapper::Channel<drobot::DataEncoders>::Ptr enc = legacy_wrapper::Channel<drobot::DataEncoders>::requestData(
      polling_timeout_);
    if (enc)
    {
      odom_msgs.left_front_travel = enc->getLeftFrontTravel();
      odom_msgs.right_front_travel = -enc->getRightFrontTravel();
      odom_msgs.left_rear_travel = enc->getLeftRearTravel();
      odom_msgs.right_rear_travel = -enc->getRightRearTravel();

      odom_msgs.left_front_speed = enc->getLeftFrontSpeed();
      odom_msgs.right_front_speed = -enc->getRightFrontSpeed();
      odom_msgs.left_rear_speed = enc->getLeftRearSpeed();
      odom_msgs.right_rear_speed = -enc->getRightRearSpeed();

      odom_msgs.left_front_angle = enc->getLeftFrontAngle();
      odom_msgs.right_front_angle = enc->getRightFrontAngle();
      odom_msgs.left_rear_angle = enc->getLeftRearAngle();
      odom_msgs.right_rear_angle = enc->getRightRearAngle();

      pub_odometry.publish(odom_msgs);

      ROS_DEBUG_STREAM("Received travel information (L:" << enc->getLeftFrontTravel() << " R:" << enc->getLeftFrontTravel() << ")");
    }
  }

  /**
  * Limit speed and angle of command to send to MCU
  */
  void DrobotHardware::limitAngleSpeed(drobot_msgs::DrobotControl &control_msg)
  {
    double large_speed = std::max(std::abs(control_msg.left_front_speed), std::abs(control_msg.right_front_speed));

    if(large_speed > max_speed_)
    {
      control_msg.left_front_speed *= max_speed_ / large_speed;
      control_msg.right_front_speed *= max_speed_ / large_speed;
      control_msg.left_rear_speed = control_msg.left_front_speed;
      control_msg.right_rear_speed = control_msg.right_front_speed;
    }

    if(control_msg.left_front_steer > left_front_steer_max_) control_msg.left_front_steer = left_front_steer_max_;
    else if(control_msg.left_front_steer < left_front_steer_min_) control_msg.left_front_steer = left_front_steer_min_;

    if(control_msg.right_front_steer > right_front_steer_max_) control_msg.right_front_steer = right_front_steer_max_;
    else if(control_msg.right_front_steer < right_front_steer_min_) control_msg.right_front_steer = right_front_steer_min_;

    if(control_msg.left_rear_steer > left_rear_steer_max_) control_msg.left_rear_steer = left_rear_steer_max_;
    else if(control_msg.left_rear_steer < left_rear_steer_min_) control_msg.left_rear_steer = left_rear_steer_min_;

    if(control_msg.right_rear_steer > right_rear_steer_max_) control_msg.right_rear_steer = right_rear_steer_max_;
    else if(control_msg.right_rear_steer < right_rear_steer_min_) control_msg.right_rear_steer = right_rear_steer_min_;

  }

} //namespace drobot_base
