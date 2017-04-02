#include <unistd.h>
#include <iostream>
#include <string.h>
#include "drobot/legacy_wrapper/crc.h"
#include "drobot/legacy_wrapper/Message.h"
#include "drobot/legacy_wrapper/Message_data.h"
#include "drobot/legacy_wrapper/Number.h"
#include "drobot/legacy_wrapper/Transport.h"
#include <ros/ros.h>

// Conditions on the below to handle compiling for nonstandard hardware
#ifdef LOGGING_AVAIL
#include "drobot/legacy_wrapper/Logger.h"
#endif

using namespace std;

namespace drobot
{
	MessageException::MessageException(const char *msg, enum errors ex_type) : Exception(msg), type(ex_type)
	{
		#ifdef LOGGING_AVAIL
		CPR_EXCEPT() << "MessageException" <<type<<": "<< message << endl << flush;
		#endif
	}

	Message::Message() : is_sent(false)
	{
		total_len = HEADER_LENGTH + CRC_LENGTH;
		memset(data, 0, MAX_MSG_LENGTH);
	}

	Message::Message(void *input, size_t msg_len) : is_sent(false)
	{
		total_len = msg_len;
		memset(data, 0, MAX_MSG_LENGTH);
		memcpy(data, input, msg_len);
	}

	Message::Message(const Message &other) : is_sent(false)
	{
		total_len = other.total_len;
		memset(data, 0, MAX_MSG_LENGTH);
		memcpy(data, other.data, total_len);
	}

	Message::Message(uint16_t type, uint8_t *payload, size_t payload_len) : is_sent(false)
	{
		/*copy in data*/
		total_len = HEADER_LENGTH + payload_len + CRC_LENGTH;
		if(total_len > MAX_MSG_LENGTH)
		{
			/* If payload is too long, the only recourse we have in constructor
			* (other than an abort()) is to truncate silently. */
			total_len = MAX_MSG_LENGTH;
			payload_len = MAX_MSG_LENGTH - HEADER_LENGTH - CRC_LENGTH;
		}
		memset(data, 0, MAX_MSG_LENGTH);
		memcpy(data + PAYLOAD_OFST, payload, payload_len);

		/* Fill header */
		data[SOH_OFST] = SOH;
		setLength(total_len - 3);
		setType(type);

		/* Generate checksum */
		uint16_t checksum = crc16(crcOffset(), CRC_INIT_VAL, data);
		utob(data + crcOffset(), 2, checksum);
	}

	Message::~Message()
	{
	  // nothing to do, actually.
	}

	void Message::send()
	{
		// We will retry up to 3 times if we receive CRC errors
		for (int i = 0; i < 2; ++i)
		{
		  try
		  {
		    Transport::instance().send(this);
		    return;
		  }
		  catch (BadAckException *ex)
		  {
		    // Any bad ack other than bad checksum needs to
		    // be thrown on
		    if (ex->ack_flag != BadAckException::BAD_CHECKSUM)
		    {
		      throw ex;
		    }
		  }
		}

		/*Make the final attempt outside the try, any exception
		*just goes straight through */
		#ifdef LOGGING_AVAIL
		CPR_WARN() << "Bad checksum twice in a row." << endl;
		#endif
		Transport::instance().send(this);
	}
	/**
	* Copies message payload into a provided buffer.
	* @param buf       The buffer to fill
	* @param buf_size  Maximum length of buf
	* @return number of bytes copied.
	*/
	size_t Message::getPayload(void *buf, size_t buf_size)
	{
		// If we don't have enough space in the buffer, don't even try
		if(getPayloadLength() > buf_size) { return 0;}

		memcpy(buf, data+PAYLOAD_OFST, getPayloadLength());
		return getPayloadLength();
	}
	/**
	* Get a pointer to the payload withing this Message's internal storage.
	* @param offset    The offset from the beginning of the payload.
	* @return a pointer to this Message's internal storage.
	*/
	uint8_t *Message::getPayloadPointer(size_t offset)
	{
		return data + PAYLOAD_OFST + offset;
	}

	uint8_t Message::getLength()
	{
		return data[LENGTH_OFST];
	}

	uint16_t Message::getType()
	{
		return btou(data + TYPE_OFST, 2);
	}

	uint16_t Message::getChecksum()
	{
		return btou(data + crcOffset(), 2);
	}

	void Message::setLength(uint8_t len)
	{
		size_t new_total_len = len +3;
		if (new_total_len > MAX_MSG_LENGTH) { return; }
		total_len = new_total_len;
		data[LENGTH_OFST] = len;
	}

	void Message::setType(uint16_t type)
	{
		utob(data + TYPE_OFST, 2, type);
	}

	/**
	* Changes the payload length of the packet.
	* Does not update packet len/~len fields or the checksum.  Call
	* makeValid() to update these fields.
	* @param len   The new payload length
	*/
	void Message::setPayloadLength(uint8_t len)
	{
		if (((size_t) (len) + HEADER_LENGTH + CRC_LENGTH) > MAX_MSG_LENGTH) { return; }
		total_len = len + HEADER_LENGTH + CRC_LENGTH;
	}

	/**
	* Set the payload of this message.  Modifies the length of the
	* message as necessary, as per setPayloadLength.
	* @see Message::setPayloadLength()
	* @param buf       Buffer containing the new payload.
	* @param buf_size  Length of buf.
	*/
	void Message::setPayload(void *buf, size_t buf_size)
	{
		if ((buf_size + HEADER_LENGTH + CRC_LENGTH) > MAX_MSG_LENGTH) { return; }
		setPayloadLength(buf_size);
		if (buf_size > getPayloadLength()) { return; }
		memcpy(data + PAYLOAD_OFST, buf, buf_size);
	}

	/**
	* Copy the complete raw content of this message to a buffer.
	* @param buf       The buffer to copy to
	* @param buf_size  The maximum length of buf
	* @return buf on success, NULL on failure
	*/

	size_t Message::toBytes(void *buf, size_t buf_size)
	{
	// If we don't have enough space in the buffer, don't even try
		if (total_len > buf_size)
		{
			return 0;
		}
		memcpy(buf, data, total_len);
		return total_len;
	}

	/**
	* Checks the consistency of this message.
	* @param whyNot    Optionally, a reason for validation failure will be
	*                  written here.
	* @param strLen    Length of the optional whyNot string
	* @return true if the message is valid, false otherwise.
	*/
	bool Message::isValid(char *whyNot, size_t strLen)
	{
		// Check SOH 
		if (data[SOH_OFST] != SOH)
		{
			if (whyNot) { strncpy(whyNot, "SOH is not present.", strLen); }
			ROS_INFO("SOH WRONG");
			return false;
		}
		// Check length is correct
		if (getLength() != (total_len - 3))
		{
			if (whyNot) { strncpy(whyNot, "Length is wrong.", strLen); }
			ROS_INFO("length WRONG");
			return false;
		}
		if (crc16(crcOffset(), CRC_INIT_VAL, this->data) != getChecksum())
		{
			if (whyNot) { strncpy(whyNot, "CRC is wrong.", strLen); }
			ROS_INFO("CRC WRONG%x", getChecksum());
			ROS_INFO("CRC WRONGget%x", crc16(crcOffset(), CRC_INIT_VAL, this->data)) ;
			return false;
		}
		return true;
	}

	/**
	* Sets SOH, STX, length, and checksum so that this message becomes valid.
	*/
	void Message::makeValid()
	{
		data[SOH_OFST] = SOH;
		data[LENGTH_OFST] = total_len - 3;
		uint16_t checksum = crc16(crcOffset(), CRC_INIT_VAL, data);
		utob(data + crcOffset(), 2, checksum);
	}

	std::ostream &Message::printMessage(std::ostream &stream)
	{
		stream << "Message" << endl;
		stream << "=======" << endl;
		stream << "Length   : " << (int) (getLength()) << endl;
		stream << "Type     : " << hex << (int) (getType()) << endl;
		stream << "Checksum : " << hex << (int) (getChecksum()) << endl;
		stream << dec;
		stream << "Raw      : ";
		printRaw(stream);
		return stream;
	}

	void Message::printRaw(std::ostream &stream)
	{
		stream << hex << uppercase;
		for (unsigned int i = 0; i < total_len; i++)
		{
			stream << static_cast<short>(data[i]) << " ";
		}
		stream << dec;
		stream << endl;
	}
	
	/**
	* Instantiates the Message subclass corresponding to the
	* type field in raw message data.
	* @param input     The raw message data to instantiate from
	* @param msg_len   The length of input.
	* @return  An instance of the correct Message subclass
	*/
	Message *Message::factory(void *input, size_t msg_len)
	{
	  uint16_t type = btou((char *) input + TYPE_OFST, 2);

	  switch (type)
	  {

	    case DATA_ENCODER:
	      return new DataEncoders(input, msg_len);

	    case DATA_SYSTEM_STATUS :
	      return new DataSystemStatus(input, msg_len);

	    default:
	      return new Message(input, msg_len);
	  } // switch getType()
	} // factory()

	Message *Message::popNext()
	{
		return Transport::instance().popNext();
	}

	Message *Message::waitNext(double timeout)
	{
		return Transport::instance().waitNext(timeout);
	}

}; // namespace drobot

std::ostream &operator<<(std::ostream &stream, drobot::Message &msg)
{
	return msg.printMessage(stream);
}