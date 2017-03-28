#ifndef MESSAGE_REQUEST_H
#define MESSAGE_REQUEST_H

#include "drobot/legacy_wrapper/Message.h"

namespace drobot
{
	class Request : public Message
	{
	public:
		Request(uint16_t type, uint16_t freq = 0);

		Request(const Request &other);
	};
}; //namespace drobot

#endif // MESSAGE_REQUEST_H