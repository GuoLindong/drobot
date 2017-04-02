#include "drobot/legacy_wrapper/Message_cmd.h"
#include <ros/ros.h>

#include <cstring>
#include <iostream>
#include <typeinfo>

using namespace std;

#include "drobot/legacy_wrapper/Number.h"
// Conditions on the below to handle compiling for nonstandard hardware
#ifdef LOGGING_AVAIL
#include "drobot/legacy_wrapper/Logger.h"
#endif

namespace drobot
{
	long CmdMessage::total_destroyed = 0;
	long CmdMessage::total_sent = 0;

	CmdMessage::~CmdMessage()
	{
	    ++total_destroyed;
	    if (!is_sent)
	    {
	    #ifdef LOGGING_AVAIL
	        CPR_WARN() << "Command message destroyed without being sent. Type: "
	            << "0x" << hex << getType() << dec 
	            << ". Total unsent: " << (total_destroyed-total_sent) << endl;
	    #endif
	    }
	    else
	    {
	      total_sent++;
	    }
	}


	SetDifferentialControl::SetDifferentialControl(
	    double left_front_speed,
	    double left_front_steer,
	    double right_front_speed,
	    double right_front_steer,
	    double left_rear_speed,
	    double left_rear_steer,
	    double right_rear_speed,
	    double right_rear_steer)
	    : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);

		ftob(getPayloadPointer(LEFT_FRONT_SPEED), 2, left_front_speed, 100);
		ftob(getPayloadPointer(LEFT_FRONT_STEER), 2, left_front_steer, 10);

		ftob(getPayloadPointer(RIGHT_FRONT_SPEED), 2, right_front_speed, 100);
		ftob(getPayloadPointer(RIGHT_FRONT_STEER), 2, right_front_steer, 10);

		ftob(getPayloadPointer(LEFT_REAR_SPEED), 2, left_rear_speed, 100);
		ftob(getPayloadPointer(LEFT_REAR_STEER), 2, left_rear_steer, 10);

		ftob(getPayloadPointer(RIGHT_REAR_SPEED), 2, right_rear_speed, 100);
		ftob(getPayloadPointer(RIGHT_REAR_STEER), 2, right_rear_steer, 10);

		setType(SET_DIFF_CTRL_CONSTS);
		makeValid();
	}

	SetDifferentialControl::SetDifferentialControl(const SetDifferentialControl &other)
	    : CmdMessage(other)
	{
	}


	SetAngleOffset::SetAngleOffset(double left_front_angle_offset, double right_front_angle_offset, double left_rear_angle_offset, double right_rear_angle_offset) : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);

		ftob(getPayloadPointer(LEFT_FRONT_ANGLE_OFFSET), 2, left_front_angle_offset, 10);
		ftob(getPayloadPointer(RIGHT_FRONT_ANGLE_OFFSET), 2, right_front_angle_offset, 10);
		ftob(getPayloadPointer(LEFT_REAR_ANGLE_OFFSET), 2, left_rear_angle_offset, 10);
		ftob(getPayloadPointer(RIGHT_REAR_ANGLE_OFFSET), 2, right_rear_angle_offset, 10);

		setType(SET_ANGLE_OFFSET);
		makeValid();
	}

	SetAngleOffset::SetAngleOffset(const SetAngleOffset &other) : CmdMessage(other)
	{
	}

};  //namespace drobot