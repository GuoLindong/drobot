#include "drobot/legacy_wrapper/Message_request.h"
#include "drobot/legacy_wrapper/Number.h"
#include <ros/ros.h>

namespace drobot
{
	Request::Request(uint16_t type, uint16_t freq) : Message()
	{
		setPayloadLength(2);
		utob(getPayloadPointer(), 2, freq);
		setType(type);

		makeValid();
	}

	Request::Request(const Request &other) : Message(other)
	{	
	}
} //namespace drobot