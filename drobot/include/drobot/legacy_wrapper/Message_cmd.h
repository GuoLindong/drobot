#ifndef MESSAGE_CMD_H
#define MESSAGE_CMD_H

#include "drobot/legacy_wrapper/Message.h"

namespace drobot
{
	class CmdMessage : public Message
	{
	private:
		static long total_destroyed;
		static long total_sent;

	public:
		CmdMessage() : Message()
		{
		}

		CmdMessage(const CmdMessage &other) : Message(other)
		{
		}

		virtual ~CmdMessage();
	};

	class CmdProcessorReset : public CmdMessage
	{
	public:
		CmdProcessorReset();

		CmdProcessorReset(const CmdProcessorReset &other);
	};

	class CmdRestoreSettings : public CmdMessage
	{
	public:
		enum restoreFlags
		{
			USER_SETTINGS = 0x1,
			FACTORY_SETTINGS = 0x2
		};
	public:
		CmdRestoreSettings(enum restoreFlags flags);

		CmdRestoreSettings(const CmdRestoreSettings &other);
	};

	class CmdStoreSettings : public CmdMessage
	{
	public:
	  CmdStoreSettings();

	  CmdStoreSettings(const CmdStoreSettings &other);
	};

	class SetAckermannOutput : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    STEERING = 0,
	    THROTTLE = 2,
	    BRAKE = 4,
	    PAYLOAD_LEN = 6
	  };

	public:
		SetAckermannOutput(double steering, double throt, double brake);

		SetAckermannOutput(const SetAckermannOutput &other);
	};

	class SetDifferentialControl : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    LEFT_FRONT_SPEED = 0,
	    LEFT_FRONT_STEER = 2,
	    RIGHT_FRONT_SPEED = 4,
	    RIGHT_FRONT_STEER = 6,
	    LEFT_REAR_SPEED = 8,
	    LEFT_REAR_STEER = 10,
	    RIGHT_REAR_SPEED = 12,
	    RIGHT_REAR_STEER = 14,
	    PAYLOAD_LEN = 16
	  };

	public:

	  SetDifferentialControl(double left_front_speed,
	      double left_front_steer,
	      double right_front_speed,
	      double right_front_steer,
	      double left_rear_speed,
	      double left_rear_steer,
	      double right_rear_speed,
	      double right_rear_steer);

	  SetDifferentialControl(const SetDifferentialControl &other);
	};

	class SetDifferentialOutput : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    LEFT = 0,
	    RIGHT = 2,
	    PAYLOAD_LEN = 4
	  };

	public:
	  SetDifferentialOutput(double left, double right);

	  SetDifferentialOutput(const SetDifferentialOutput &other);
	};

	class SetDifferentialSpeed : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    LEFT_SPEED = 0,
	    RIGHT_SPEED = 2,
	    LEFT_ACCEL = 4,
	    RIGHT_ACCEL = 6,
	    PAYLOAD_LEN = 8
	  };

	public:
	  SetDifferentialSpeed(double left_speed, double right_speed, double left_accel, double right_accel);

	  SetDifferentialSpeed(const SetDifferentialSpeed &other);
	};

	class SetGear : public CmdMessage
	{
	public:
	  SetGear(uint8_t gear);

	  SetGear(const SetGear &other);
	};

	class SetAngleOffset : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    LEFT_FRONT_ANGLE_OFFSET = 0,
	    RIGHT_FRONT_ANGLE_OFFSET = 2,
	    LEFT_REAR_ANGLE_OFFSET = 4,
	    RIGHT_REAR_ANGLE_OFFSET = 6,
	    PAYLOAD_LEN = 8
	  };

	public:
	  SetAngleOffset(double left_front_angle_offset, double right_front_angle_offset, double left_rear_angle_offset, double right_rear_angle_offset);

	  SetAngleOffset(const SetAngleOffset &other);
	};

	class SetMaxSpeed : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    MAX_FWD = 0,
	    MAX_REV = 2,
	    PAYLOAD_LEN = 4
	  };

	public:
	  SetMaxSpeed(double max_fwd, double max_rev);

	  SetMaxSpeed(const SetMaxSpeed &other);
	};

	class SetPlatformName : public CmdMessage
	{
	public:
	  SetPlatformName(const char *name);

	  SetPlatformName(const SetPlatformName &other);
	};

	class SetPlatformTime : public CmdMessage
	{
	public:
	  SetPlatformTime(uint32_t time);

	  SetPlatformTime(const SetPlatformTime &other);
	};

	class SetSafetySystem : public CmdMessage
	{
	public:
	  SetSafetySystem(uint16_t flags);

	  SetSafetySystem(const SetSafetySystem &other);
	};

	class SetTurn : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    TRANSLATIONAL = 0,
	    TURN_RADIUS = 2,
	    TRANS_ACCEL = 4,
	    PAYLOAD_LEN = 6
	  };

	public:
	  SetTurn(double trans, double rad, double accel);

	  SetTurn(const SetTurn &other);
	};

	class SetVelocity : public CmdMessage
	{
	public:
	  enum payloadOffsets
	  {
	    TRANSLATIONAL = 0,
	    ROTATIONAL = 2,
	    TRANS_ACCEL = 4,
	    PAYLOAD_LEN = 6
	  };

	public:
	  SetVelocity(double trans, double rot, double accel);

	  SetVelocity(const SetVelocity &other);
	};


}; // namespace drobot

#endif //MESSAGE_CMD_H