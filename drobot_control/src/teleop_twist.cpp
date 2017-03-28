#include "drobot_control/teleop_twist.h"

teleop_twist::teleop_twist()
{
	memset(js_axes, 0, sizeof(js_axes));
    memset(js_buttons, 0, sizeof(js_buttons));

    sub_js = nh.subscribe("joy",1,&teleop_twist::joyStick_callback,this);
    cmd_pub = nh.advertise<drobot_msgs::DrobotControl>("command_info", 5);
    stopMode(control_msgs);
}


void teleop_twist::teleopLoop()
{
	//按键X 原地转向
    if(js_buttons[2]==1){
      ROS_INFO("FOUR WHEEL STEERING!");
      control_msgs.point_turn_mode = true;
      control_msgs.gear_mode = 1;
    }

    //按键A 四轮转向
    if(js_buttons[0]==1){
      ROS_INFO("FRONT WHEEL STEERING!");
      control_msgs.point_turn_mode = false;
      control_msgs.gear_mode = 1;
    }
    //按键B
    if(js_buttons[1]==1){
      ROS_INFO("FOUR WHEEL STEERING!");
      control_msgs.point_turn_mode = false;
      control_msgs.gear_mode = 2;
    }

	//使能关，按键RB
    if(js_buttons[5]==1) {
        ROS_INFO("DRIVER DISABLED!");
        control_msgs.stop_mode = true;//右键停
        control_msgs.gear_mode = 1;
    }
    //动作使能开.按键LB
    if(js_buttons[4]==1){//LB
        ROS_INFO("DRIVER ENABLED!");
        control_msgs.stop_mode = false; //左键行
        control_msgs.gear_mode = 1;
    } 

    //动作使能开
    if(!control_msgs.stop_mode)
    {

      if(control_msgs.point_turn_mode == true)
      {
        angleConvertStopAndTurn(control_msgs , js_axes[1]);
          
      }
      else
      {
        angleSpeedConvert(control_msgs, js_axes[1], js_axes[3]);
      }
    }
    else{
      stopMode(control_msgs);
    }
    cmd_pub.publish(control_msgs);
}


//手柄控制回调函数
void teleop_twist::joyStick_callback(const sensor_msgs::Joy &msg)
{
  for (int i = 0; i < 8; i++){
    js_axes[i] = msg.axes[i];
  } 
  for (int i = 0; i < 11; i++){
    js_buttons[i] = msg.buttons[i];
  }
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_twist");

	teleop_twist teleop_turtle;
  	//signal(SIGINT,quit);
  	ros::Rate loop_rate(100);
  	while (ros::ok()){
    	teleop_turtle.teleopLoop();
    	ros::spinOnce(); 
    	loop_rate.sleep();  
  	}
  
  	return(0);
}