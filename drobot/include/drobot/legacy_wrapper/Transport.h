#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <list>
#include <iostream>

#include "drobot/legacy_wrapper/Message.h"
#include "drobot/legacy_wrapper/Exception.h"

namespace drobot
{
	class  TransportException : public Exception
	{
	public:
		enum  errors
		{
			ERROR_BASE,
			NOT_CONFIGURED,
			CONFIGURE_FAIL,
			UNACKNOWLEDGED_SEND,
			BAD_ACK_RESULT
		};
	public:
		enum  errors type;
		TransportException(const char *msg, enum errors ex_type = ERROR_BASE);	
	};

	class BadAckException : public TransportException
	{
	public:
	  enum ackFlags
	  {
	    BAD_CHECKSUM = 0x01,
	    BAD_TYPE = 0x02,
	    BAD_FORMAT = 0x04,
	    RANGE = 0x08,
	    NO_BANDWIDTH = 0x10,
	    OVER_FREQ = 0x20,
	    OVER_SUBSCRIBE = 0x40
	  } ack_flag;

	  BadAckException(unsigned int flag);
	};

	class  Transport
	{
	public:
		enum  counterTypes
		{
			GARBLE_BYTES, // bytes with no SOH / bad length
			INVALID_MSG,  // bad format / CRC wrong
			IGNORED_ACK,  // ack we didn't care about
			QUEUE_FULL,   // dropped msg because of overfull queue
			NUM_COUNTERS  // end of list, not actual counter
			
		};
		static const char *counter_names[NUM_COUNTERS]; // N.B: must be updated with counterTypes

	private:
		bool configured;
		void *serial;
		int retries;

		static const int RETRY_DELAY_MS = 200;

		std::list<Message *> rx_queue;
		static const size_t MAX_QUEUE_LEN = 10000;

		unsigned long counters[NUM_COUNTERS];	

	private:
		Message *rxMessage();

		Message *getAck();

		void enqueueMessage(Message *msg);

		int openComm(const char *device);

		int closeComm();

		void resetCounters();

	protected:
		Transport();

		~Transport();

	public:
		static Transport &instance();

		void configure(const char *device, int retries);

		bool isConfigured()
		{
		  return configured;
		}

		int close();

		void poll();

		void send(Message *m);

		Message *popNext();

		Message *popNext(enum MessageTypes type);

		Message *waitNext(double timeout = 0.0);

		Message *waitNext(enum MessageTypes type, double timeout = 0.0);

		void flush(std::list<Message *> *queue = 0);

		void flush(enum MessageTypes type, std::list<Message *> *queue = 0);

		unsigned long getCounter(enum counterTypes counter)
		{
		  return counters[counter];
		}

		void printCounters(std::ostream &stream = std::cout);
	};
};

#endif // TRANSPORT_H