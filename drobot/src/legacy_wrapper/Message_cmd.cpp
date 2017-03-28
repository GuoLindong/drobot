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

	CmdProcessorReset::CmdProcessorReset() : CmdMessage()
	{
		setPayloadLength(2);
		utob(getPayloadPointer(), 2, (uint16_t) 0x3A18);
		setType(CMD_PROCESSOR_RESET);
		makeValid();
	}

	CmdProcessorReset::CmdProcessorReset(const CmdProcessorReset &other) : CmdMessage(other)
	{
	}

	CmdRestoreSettings::CmdRestoreSettings(enum restoreFlags flags) : CmdMessage()
	{
		setPayloadLength(3);

		utob(getPayloadPointer(), 2, (uint16_t) (0x3a18));
		*getPayloadPointer(2) = (uint8_t) (flags);
		setType(CMD_RESTORE_SETTINGS);

		makeValid();
	}

	CmdRestoreSettings::CmdRestoreSettings(const CmdRestoreSettings &other) : CmdMessage(other)
	{
	}

	CmdStoreSettings::CmdStoreSettings() : CmdMessage()
	{
		setPayloadLength(2);
		utob(getPayloadPointer(), 2, (uint16_t) (0x3a18));
		setType(CMD_STORE_SETTINGS);

		makeValid();
	}

	CmdStoreSettings::CmdStoreSettings(const CmdStoreSettings &other) : CmdMessage(other)
	{
	}

	SetAckermannOutput::SetAckermannOutput(double steer, double throt, double brake) : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);

		ftob(getPayloadPointer(STEERING), 2, steer, 100);
		ftob(getPayloadPointer(THROTTLE), 2, throt, 100);
		ftob(getPayloadPointer(BRAKE), 2, brake, 100);

		setType(SET_ACKERMANN_SETPT);
		makeValid();
	}

	SetAckermannOutput::SetAckermannOutput(const SetAckermannOutput &other) : CmdMessage(other)
	{
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

	SetDifferentialOutput::SetDifferentialOutput(double left, double right) : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);
		ftob(getPayloadPointer(LEFT), 2, left, 100);
		ftob(getPayloadPointer(RIGHT), 2, right, 100);

		setType(SET_DIFF_WHEEL_SETPTS);
		makeValid();
	}

	SetDifferentialOutput::SetDifferentialOutput(const SetDifferentialOutput &other)
	    : CmdMessage(other)
	{
	}

	SetDifferentialSpeed::SetDifferentialSpeed(double left_speed, double right_speed,
	    double left_accel, double right_accel)
	    : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);

		ftob(getPayloadPointer(LEFT_SPEED), 2, left_speed, 100);
		ftob(getPayloadPointer(LEFT_ACCEL), 2, left_accel, 100);
		ftob(getPayloadPointer(RIGHT_SPEED), 2, right_speed, 100);
		ftob(getPayloadPointer(RIGHT_ACCEL), 2, right_accel, 100);

		setType(SET_DIFF_WHEEL_SPEEDS);
		makeValid();
	}

	SetDifferentialSpeed::SetDifferentialSpeed(const SetDifferentialSpeed &other) : CmdMessage(other)
	{
	}

	SetGear::SetGear(uint8_t gear) : CmdMessage()
	{
		setPayloadLength(1);
		getPayloadPointer()[0] = gear;
		setType(SET_GEAR_SETPOINT);
		makeValid();
	}

	SetGear::SetGear(const SetGear &other) : CmdMessage(other)
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

	SetMaxSpeed::SetMaxSpeed(double max_fwd, double max_rev) : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);

		ftob(getPayloadPointer(MAX_FWD), 2, max_fwd, 100);
		ftob(getPayloadPointer(MAX_REV), 2, max_rev, 100);

		setType(SET_MAX_SPEED);
		makeValid();
	}

	SetMaxSpeed::SetMaxSpeed(const SetMaxSpeed &other) : CmdMessage(other)
	{
	}

	SetPlatformName::SetPlatformName(const char *name) : CmdMessage()
	{
		size_t cpy_len = strlen(name);
		size_t max_len = MAX_MSG_LENGTH - HEADER_LENGTH - CRC_LENGTH - 1 /* for size field */;
		if (cpy_len > max_len) { cpy_len = max_len; }

		setPayloadLength(cpy_len + 1);
		getPayloadPointer()[0] = cpy_len;
		memcpy(getPayloadPointer(1), name, cpy_len);

		setType(SET_PLATFORM_NAME);

		makeValid();
	}

	SetPlatformName::SetPlatformName(const SetPlatformName &other) : CmdMessage(other)
	{
	}

	SetPlatformTime::SetPlatformTime(uint32_t time) : CmdMessage()
	{
		setPayloadLength(4);
		utob(getPayloadPointer(), 4, time);
		setType(SET_PLATFORM_TIME);
		makeValid();
	}

	SetPlatformTime::SetPlatformTime(const SetPlatformTime &other) : CmdMessage(other)
	{
	}

	SetSafetySystem::SetSafetySystem(uint16_t flags) : CmdMessage()
	{
		setPayloadLength(2);
		utob(getPayloadPointer(), 2, flags);
		setType(SET_SAFETY_SYSTEM);
		makeValid();
	}

	SetSafetySystem::SetSafetySystem(const SetSafetySystem &other) : CmdMessage(other)
	{
	}

	SetTurn::SetTurn(double trans, double rad, double accel) : CmdMessage()
	{
		setPayloadLength(PAYLOAD_LEN);

		ftob(getPayloadPointer(TRANSLATIONAL), 2, trans, 100);
		ftob(getPayloadPointer(TURN_RADIUS), 2, rad, 100);
		ftob(getPayloadPointer(TRANS_ACCEL), 2, accel, 100);

		setType(SET_TURN_SETPT);
		makeValid();
	}

	SetTurn::SetTurn(const SetTurn &other) : CmdMessage(other)
	{
	}

	SetVelocity::SetVelocity(double trans, double rot, double accel) : CmdMessage()
	{
	  setPayloadLength(PAYLOAD_LEN);

	  ftob(getPayloadPointer(TRANSLATIONAL), 2, trans, 100);
	  ftob(getPayloadPointer(ROTATIONAL), 2, rot, 100);
	  ftob(getPayloadPointer(TRANS_ACCEL), 2, accel, 100);

	  setType(SET_VELOCITY_SETPT);
	  makeValid();
	}

	SetVelocity::SetVelocity(const SetVelocity &other) : CmdMessage(other)
	{
	}

};  //namespace drobot