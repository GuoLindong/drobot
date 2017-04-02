#ifndef MESSAGE_H
#define MESSAGE_H

#include <iostream>
#include <cstdlib>
#include <stdint.h>

#include "drobot/legacy_wrapper/Exception.h"

namespace drobot
{
	class MessageException : public Exception
	{
	public:
	  enum errors
	  {
	    ERROR_BASE = 0,
	    INVALID_LENGTH
	  };

	public:
	  enum errors type;

	  MessageException(const char *msg, enum errors ex_type = ERROR_BASE);
	};

	class Message
	{
	public:
		static const size_t MAX_MSG_LENGTH = 256;

	protected:
		static const size_t CRC_LENGTH = 2;
		static const uint16_t CRC_INIT_VAL = 0xFFFF;
		static const size_t HEADER_LENGTH = 4;

		enum dataoffsets
		{
			SOH_OFST = 0,
			LENGTH_OFST = 1,
			TYPE_OFST = 2,
			PAYLOAD_OFST = 4
		};

		uint8_t data[MAX_MSG_LENGTH];
		// Total length (incl. full header & checksum)
		size_t total_len;

		// Whether this Message has ever been sent by the Transport()
		// (Updated by Transport::send())
		bool is_sent;

		friend class Transport; // Allow Transport to read data and total_len directly

	public:
		static const size_t MIN_MSG_LENGTH = HEADER_LENGTH + CRC_LENGTH;
		static const uint8_t SOH = 0xAA;

	protected:
		size_t crcOffset()
		{
			return total_len - CRC_LENGTH;
		};

		void setLength(uint8_t len);

		void setType(uint16_t type);

		uint8_t *getPayloadPointer(size_t offset = 0);

		void setPayload(void *buf, size_t buf_size);

		void setPayloadLength(uint8_t len);

		void makeValid();

	public:
		Message();

		Message(void *input, size_t msg_len);

		Message(const Message &other);

		Message(uint16_t type, uint8_t *payload, size_t payload_len);

		virtual ~Message();

		void send();

		uint8_t getLength(); // as reported by packet length field.
		
		uint16_t getType();

		uint16_t getChecksum();

		size_t getPayloadLength()
		{
			return total_len - HEADER_LENGTH - CRC_LENGTH;
		}

		size_t getPayload(void *buf, size_t max_size);

		size_t getTotalLength()
		{
			return total_len;
		}

		size_t toBytes(void *buf, size_t buf_size);

		bool isValid(char *whyNot = NULL, size_t strLen = 0);

		bool isCommand()
		{
			return (0x00FF & getType()) < 0x40;
		}

		bool isRequest()
		{
			return ((0x00FF & getType()) >= 0x40) && ((0x00FF & getType()) < 0x80);
		}

		bool isData()
		{
			return ((0x00FF & getType()) >= 0x80) && ((0x00FF & getType())< 0xC0);
		}

		virtual std::ostream &printMessage(std::ostream &stream = std::cout);

		void printRaw(std::ostream &stream = std::cout);

		static Message *factory(void *input, size_t msg_len);

		static Message *popNext();

		static Message *waitNext(double timeout = 0.0);

	}; // class Message

	enum  MessageTypes
	{
		/*
		 * Set commands
		 */

		 SET_DIFF_CTRL_CONSTS = 0x0120,
		SET_ANGLE_OFFSET = 0x0220,

		/*
		 * Request commands
		 */
		
		REQUEST_ENCODER = 0x0140,
		REQUEST_SYSTEM_STATUS = 0x0240,
		/*
		 * Data commands
		 */
		
		DATA_ENCODER = 0x0180,
		DATA_SYSTEM_STATUS = 0x0280
	}; // enum MessageTypes

}; // namespace drobot

std::ostream &operator<<(std::ostream &stream, drobot::Message &msg);

#endif // MESSAGE_H
