#include <string.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <ctime>

#include "drobot/legacy_wrapper/Transport.h"
#include "drobot/legacy_wrapper/Number.h"
#include "drobot/legacy_wrapper/Message.h"
#include "drobot/legacy_wrapper/Message_request.h"
#include "drobot/legacy_wrapper/Message_cmd.h"
#include "drobot/legacy_wrapper/serial.h"
#include "drobot/legacy_wrapper/Logger.h"

#include <ros/ros.h>

using namespace std;

namespace drobot
{
	const char *Transport::counter_names[] = {
	    "Garbled bytes",
	    "Invalid messages",
	    "Ignored acknowledgment",
	    "Message queue overflow"
	};

	  TransportException::TransportException(const char *msg, enum errors ex_type)
	      : Exception(msg), type(ex_type)
	  {
	    if (msg)
	    {
	      CPR_EXCEPT() << "TransportException " << (int) type << ": " << message << endl << std::flush;
	    }
	  }

	  BadAckException::BadAckException(unsigned int flag) :
	      TransportException(NULL, TransportException::BAD_ACK_RESULT),
	      ack_flag((enum ackFlags) flag)
	  {
	    switch (ack_flag)
	    {
	      case BAD_CHECKSUM:
	        message = "Bad checksum";
	        break;
	      case BAD_TYPE:
	        message = "Bad message type";
	        break;
	      case BAD_FORMAT:
	        message = "Bad message format";
	        break;
	      case RANGE:
	        message = "Range error";
	        break;
	      case OVER_FREQ:
	        message = "Requested frequency too high";
	        break;
	      case OVER_SUBSCRIBE:
	        message = "Too many subscriptions";
	        break;
	      default:
	        message = "Unknown error code.";
	        break;
	    };
	    stringstream ss;

	    CPR_EXCEPT() << "BadAckException (0x" << hex << flag << dec << "): " << message << endl << flush;
	  }

	#define CHECK_THROW_CONFIGURED() \
	    do { \
	        if( ! configured ) { \
	            throw new TransportException("Transport not configured", TransportException::NOT_CONFIGURED); \
	        } \
	    } while( 0 )

    /**
    * Transport singleton instance accessor.
    * @return  The Transport singleton instance.
    */
      Transport &Transport::instance()
      {
        static Transport instance;
        return instance;
      }

    /**
    * Constructs an unconfigured transport instance.
    */
      Transport::Transport() :
          configured(false),
          serial(0),
          retries(0)
      {
        for (int i = 0; i < NUM_COUNTERS; ++i)
        {
          counters[i] = 0;
        }
      }

      Transport::~Transport()
      {
        close();
      }

	/**
	* Configure this Transport for communication.
	* If this Transport is already configured, it will be closed and reconfigured.
	* The RX buffer and Message queue will be flushed.
	* Counters will be reset.
	* @param device    The device to communicate over.  (Currently, must be serial)
	* @param retries   Number of times to resend an unacknowledged message.
	* @throws TransportException if configuration fails
	* @post Transport becomes configured.
	*/
	  void Transport::configure(const char *device, int retries)
	  {
	    if (configured)
	    {
	      // Close serial
	      close();
	    }

	    // Forget old counters
	    resetCounters();

	    this->retries = retries;

	    if (!openComm(device))
	    {
	      configured = true;
	    }
	    else
	    {
	      throw new TransportException("Failed to open serial port", TransportException::CONFIGURE_FAIL);
	    }
	  }

	  /**
	  * Close this Transport.
	  * @return Zero on success, nonzero otherwise
	  * @post Tranport will be unconfigured, regardless of success/failure.
	  */
	    int Transport::close()
	    {
	      int retval = 0;
	      if (configured)
	      {
	        flush();
	        retval = closeComm();
	      }
	      configured = false;
	      return retval;
	    }

	    /**
	    * Opens a serial port with the default configuration
	    * (115200 bps, 8-N-1), using the device specified in the constructor
	    */
	      int Transport::openComm(const char *device)
	      {
	        int tmp = OpenSerial(&(this->serial), device);
	        if (tmp < 0)
	        {
	          return -1;
	        }
	        tmp = SetupSerial(this->serial);
	        if (tmp < 0)
	        {
	          return -2;
	        }
	        return 0;
	      }

	      /**
	      * Closes the associated serial port
	      */
        int Transport::closeComm()
        {
          CloseSerial(this->serial);
          //serial = 0;
          return 0;
        }

        /**
        * Non-blocking message receive function.
        * !!! Absolutely not reentrant !!!
        * !!! Keeps internal static state !!!
        * @return  A pointer to a dynamically allocated message, if one has been received
        *          this call.  Null if no complete message has been received.  Bad data
        *          are silently eaten.
        */
        Message *Transport::rxMessage()
        {
          /* Each time this function is called, any available characters are added
           * to the receive buffer.  A new Message is created and returned when 
           * a complete message has been received (the message may be aggregated
           * from data received over multiple calls) */
          static char rx_buf[Message::MAX_MSG_LENGTH];
          static size_t rx_inx = 0;
          static size_t msg_len = 0;

          if (!rx_inx) { memset(rx_buf, 0xba, Message::MAX_MSG_LENGTH); }

          /* Read in and handle characters, one at a time. 
           * (This is a simple state machine, using 'rx_inx' as state) */
          while (ReadData(serial, rx_buf + rx_inx, 1) == 1)
          {
            switch (rx_inx)
            {

              /* Waiting for SOH */
              case 0:
                if ((uint8_t) (rx_buf[0]) == (uint8_t) (Message::SOH))
                {
                  rx_inx++;
                }
                else { counters[GARBLE_BYTES]++; }
                break;

                /* Waiting for length */
              case 1:
                rx_inx++;
                msg_len = rx_buf[1] + 3;
                break;

    //             /* Waiting for ~length */
    //           case 2:
    //             rx_inx++;
                

    //             // /* Check for valid length */
    //             // if (static_cast<unsigned char>(rx_buf[1] ^ rx_buf[2]) != 0xFF ||
    //             //     (msg_len < Message::MIN_MSG_LENGTH))
    //             // {
    //             //   counters[GARBLE_BYTES] += rx_inx;
    //             //   rx_inx = 0;
    //             // }
				// ROS_INFO("CASE2%d", msg_len);
    //             break;

                //case 9:
                //case 10:
                //    cout << hex << " " << (unsigned int)(rx_buf[rx_inx]);
                //    if(rx_inx==10) cout << endl;

                /* Waiting for the rest of the message */
              default:
                rx_inx++;
                if (rx_inx < msg_len) { break; }
                /* Finished rxing, reset this state machine and return msg */
                rx_inx = 0;
                 // for(int K=0; K<39; K++)
          	      //       {
          	      //          	ROS_INFO("Transport%d%X",K,rx_buf[K]);
          	      //       }
                Message *msg = Message::factory(rx_buf, msg_len);
                // ROS_INFO("msg receive%x", msg->getType());
                return msg;

            } // switch( rx_inx )
          } // while( get character )
                       //  for(int K=0; K<39; K++)
          	            // {
          	            //    	ROS_INFO("Transport%d%X",K,rx_buf[K]);
          	            // }

          // Breaking out of loop indicates end of available serial input
          return NULL;
        }

        /**
        * Read data until an ack message is found.
        * Any data messages received by this function will be queued.
        * @return  The next ack message, if one is read.
        *          Null if no ack message has been read yet.
        */
	      Message *Transport::getAck()
	      {
	        Message *msg = NULL;

	        while ((msg = rxMessage()))
	        {
	          //ROS_INFO("ack null");
	          /* Queue any data messages that turn up */
	          if (msg->isData())
	          {
	            ROS_INFO("enqueue");
	            enqueueMessage(msg);
	            continue;
	          }

	          /* Drop invalid messages */
	          if (!msg->isValid())
	          {
	            ROS_INFO("not valid");
	            ++counters[INVALID_MSG];
	            delete msg;
	            continue;
	          }

	          return msg;
	        }
	        //ROS_INFO("ack null");
	        return NULL;
	      }

	     /**
	     * Add a Message to the Message queue.
	     * Checks Message validity, and drops invalid messages.
	     * Trims queue down to size if it gets too big.
	     * @param msg   The message to enqueue.
	     */
	       void Transport::enqueueMessage(Message *msg)
	       {
	         /* Reject invalid messages */
	         if (!msg->isValid())
	         {
	           ++counters[INVALID_MSG];
	           delete msg;
	           return;
	         }

	         // Enqueue
	         rx_queue.push_back(msg);

	         /* Drop the oldest messages if the queue has overflowed */
	         while (rx_queue.size() > MAX_QUEUE_LEN)
	         {
	           ++counters[QUEUE_FULL];
	           delete rx_queue.front();
	           rx_queue.pop_front();
	         }
	       } 

	       /**
	       * Public function which makes sure buffered messages are still being read into
	       * the internal buffer. A compromise between forcing a thread-based implementation
	       * and blocking on results. This could be placed into a separate thread, but will
	       * need to be wrapped for thread safety
	       */
	         void Transport::poll()
	         {
	           CHECK_THROW_CONFIGURED();

	           Message *msg = NULL;
	           // ROS_INFO(" in poll");
	           while ((msg = rxMessage()))
	           {
	             ROS_INFO(" already in poll");
	             /* We're not waiting for acks, so drop them */
	             if (!msg->isData())
	             {
	               ++counters[IGNORED_ACK];
	               delete msg;
	               continue;
	             }

	             // Message is good, queue it.
	             enqueueMessage(msg);
	           }
	         }

	         /**
	         * Send a message.
	         * Waits for the firmware to acknowlge and resends the packet
	         * a few timew if not acknowledged.
	         * @param m The message to send
	         * @throw   Transport::Exception if never acknowledged.
	         */
	           void Transport::send(Message *m)
	           {
	             CHECK_THROW_CONFIGURED();

	             char skip_send = 0;
	             Message *ack = NULL;
	             int transmit_times = 0;
	             short result_code;

	             poll();

	             while (1)
	             {
	               // We have exceeded our retry numbers
	               if (transmit_times > this->retries)
	               {
	                 break;
	               }
	               // Write output
	               if (!skip_send)
	               {  
	               		WriteData(serial, (char *) (m->data), m->total_len);
	            
	               		  // for(int K=0; K<10; K++)
          	         //    {
          	         //       	ROS_INFO("Transport%d%X",K,m->data[K]);
          	         //    }

	               }


	               // Wait up to 100 ms for ack
	               for (int i = 0; i < RETRY_DELAY_MS; ++i)
	               {
	                 usleep(1000);
	                 if ((ack = getAck())) { break; }
	               }

	               // No message - resend
	               if (ack == NULL)
	               {
	                 skip_send = 0;
	                 //cout << "No message received yet" << endl;

	                 transmit_times++;
	                 continue;
	               }

	               // Check result code
	               // If the result code is bad, the message was still transmitted
	               // successfully
	               result_code = btou(ack->getPayloadPointer(), 2);
	               if (result_code > 0)
	               {
	                 throw new BadAckException(result_code);
	                 ROS_INFO("BADRESULT");
	               }
	               else
	               {
	                 // Everything's good - return
	               	 ROS_INFO("GOODRESULT");
	                 break;
	               }
	               // Other failure
	               transmit_times++;
	             }
	             if (ack == NULL)
	             {
	             	ROS_INFO("ACKNULL");
	               throw new TransportException("Unacknowledged send", TransportException::UNACKNOWLEDGED_SEND);
	             }
	             // ROS_INFO("DELETE");
	             delete ack;

	             m->is_sent = true;
	           }

	    /**
	    * Removes the oldest Message from the Message queue and returns it.
	    * All data waiting in the input buffer will be read and queued.
	    * @return  The oldest message in the queue.  This Message is removed
	    *          from the queue.  It is dynamically allocated; the caller
	    *          is responsible for freeing it.
	    *          Null if no Messages are currently queued.
	    */
	    Message *Transport::popNext()
	    {
	      CHECK_THROW_CONFIGURED();

	      poll();  // empty the current serial RX queue.

	      if (rx_queue.empty()) { return NULL; }

	      Message *next = rx_queue.front();
	      rx_queue.pop_front();
	      return next;
	    }

	    /**
	    * Finds the oldest message of a specific type in the Message queue, removes
	    * it, and returns it.  Older messages of the wrong type will be left in
	    * the queue.
	    * All data waiting in the input buffer will be read and queued.
	    * @return  The oldest message of the correct type in the queue.  This Message
	    *          is removed from the queue.  It is dynamically allocated; the caller
	    *          is responsible for freeing it.
	    *          Null if no Messages are currently queued.
	    */
	      Message *Transport::popNext(enum MessageTypes type)
	      {
	        CHECK_THROW_CONFIGURED();

	        poll(); // empty the current RX queue

	        Message *next;
	        list<Message *>::iterator iter;
	        for (iter = rx_queue.begin(); iter != rx_queue.end(); ++iter)
	        {
	          // ROS_INFO("type test%x\n", (*iter)->getType());
	          // ROS_INFO("type need%x\n", type);
	          if ((*iter)->getType() == type)
	          {
	            next = *iter;
	            rx_queue.erase(iter);
	            return next;
	          }
	        }
	        //ROS_INFO("DOES NOT HAVE NEXT");
	        return NULL;
	      }

	    /**
	    * Fetch a message, blocking if there are no messages currently available.
	    * @param timeout   Maximum time to block, in seconds.
	    *                  Actual resolution is system dependent
	    *                  A timeout of 0.0 indicates no timeout.
	    * @return  A message.  Null if the timeout elapses. */
      Message *Transport::waitNext(double timeout)
      {
        CHECK_THROW_CONFIGURED();

        double elapsed = 0.0;
        while (true)
        {
          /* Return a message if it's turned up */
          poll();
          if (!rx_queue.empty()) { return popNext(); }

          /* Check timeout */
          if ((timeout != 0.0) && (elapsed > timeout))
          {
            // If we have a timeout set, and it has elapsed, exit.
            return NULL;
          }

          // Wait a ms before retry
          usleep(1000);
          elapsed += 0.001;
        }
      }

      /**
      * Fetch a particular type of message, blocking if one isn't available.
      * @param type      The type of message to fetch
      * @param timeout   Maximum time to block, in seconds.
      *                  Actual resolution is system dependent
      *                  A timeout of 0.0 indicates no timeout.
      * @return A message of the requested type.  Nul if the timeout elapses.
      */
      Message *Transport::waitNext(enum MessageTypes type, double timeout)
      {
        CHECK_THROW_CONFIGURED();

        double elapsed = 0.0;
        Message *msg;

        while (true)
        {
          /* Check if the message has turned up
           * Since we're blocking, not doing anything useful anyway, it doesn't
           * really matter that we're iterating the entire message queue every spin. */
          poll();
          msg = popNext(type);
          if (msg) {  return msg; }

          /* Check timeout */
          if ((timeout != 0.0) && (elapsed > timeout))
          {
            // If a timeout is set and has elapsed, fail out.
            // ROS_INFO("LONG TIME");
            return NULL;
          }

          // Wait a ms  before retry
          usleep(1000);
          elapsed += 0.001;
        }
      }

      /**
      * Empties the serial buffer and the entire message queue.
      * Optionally, the queue may be emptied into a provided list.
      * @param queue If not null, the entire message queue will be
      *              added to this list, oldest first.
      *              If null, messages will be deleted.
      */
    void Transport::flush(list<Message *> *queue)
    {
      CHECK_THROW_CONFIGURED();

      poll(); // flush serial buffer

      /* Either delete or move all elements in the queue, depending
       * on whether a destination list is provided */
      list<Message *>::iterator iter;
      for (iter = rx_queue.begin(); iter != rx_queue.end(); ++iter)
      {
        if (queue)
        {
          queue->push_back(*iter);
        }
        else
        {
          delete *iter;
        }
      }

      rx_queue.clear();
    }

    /**
    * Empties the serial buffer and removes all messages of a given type
    * from the message queue.  Optionally, removed messages may be added
    * to a provided list.
    * @param type  The type of message to flush out.
    * @param queue If not null, all messages of the correct type will be
    *              added to this list, oldest first.
    *              If null, messages of the correct type will be deleted.
    */
      void Transport::flush(enum MessageTypes type, list<Message *> *queue)
      {
        CHECK_THROW_CONFIGURED();

        poll();

        list<Message *>::iterator iter = rx_queue.begin();
        while (iter != rx_queue.end())
        {
          if ((*iter)->getType() == type)
          {
            /* Element is of flush type.  If there's a destination
             * list, move it.  Otherwise, destroy it */
            if (queue)
            {
              queue->push_back(*iter);
            }
            else
            {
              delete *iter;
            }
            // This advances to the next element in the queue:
            iter = rx_queue.erase(iter);
          }
          else
          {
            // Not interested in this element.  Next!
            iter++;
          }
        }
      }

     /**
     * Prints a nice list of counter values
     */
       void Transport::printCounters(ostream &stream)
       {
         stream << "Transport Counters" << endl;
         stream << "==================" << endl;

         size_t longest_name = 0;
         size_t cur_len = 0;
         for (int i = 0; i < NUM_COUNTERS; ++i)
         {
           cur_len = strlen(counter_names[i]);
           if (cur_len > longest_name) { longest_name = cur_len; }
         }

         for (int i = 0; i < NUM_COUNTERS; ++i)
         {
           cout.width(longest_name);
           cout << left << counter_names[i] << ": " << counters[i] << endl;
         }

         cout.width(longest_name);
         cout << left << "Queue length" << ": " << rx_queue.size() << endl;
       } 

       /**
       * Wipes out counters
       */
         void Transport::resetCounters()
         {
           for (int i = 0; i < NUM_COUNTERS; ++i)
           {
             counters[i] = 0;
           }
         }

}; //namespace drobot