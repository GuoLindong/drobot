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


}; // namespace drobot

#endif //MESSAGE_CMD_H