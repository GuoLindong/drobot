#ifndef DROBOT_BASE_LEGACY_WRAPPER_H
#define DROBOT_BASE_LEGACY_WRAPPER_H

#include "drobot/legacy_wrapper/drobot.h"
#include "boost/type_traits/is_base_of.hpp"
#include "ros/ros.h"
#include "drobot_msgs/DrobotControl.h"

namespace
{
	const uint16_t UNSUBSCRIBE = 0xFFFF;
}

namespace legacy_wrapper
{

  void connect(std::string port);

  void reconnect();

  void configureLimits(double left_front_angle_offset, double right_front_angle_offset, double left_rear_angle_offset, double right_rear_angle_offset);

  void controlSpeed(drobot_msgs::DrobotControl &msg);

  template<typename T>
  struct Channel
  {

    typedef boost::shared_ptr<T> Ptr;
    typedef boost::shared_ptr<const T> ConstPtr;
    BOOST_STATIC_ASSERT_MSG(
      (boost::is_base_of<drobot::Message, T>::value),
      "T must be a descendant of drobot::Message"
    );

    static Ptr getLatest(double timeout)
    {
      T *latest = 0;

      // Iterate over all messages in queue and find the latest
      while (T *next = T::popNext())
      {
        if (latest)
        {
          delete latest;
          latest = 0;
        }
        latest = next;
      }

      // If no messages found in queue, then poll for timeout until one is received
      if (!latest)
      {
        latest = T::waitNext(timeout);
      }

      // If no messages received within timeout, make a request
      if (!latest)
      {
        return requestData(timeout);
      }

      return Ptr(latest);
    }

    static Ptr requestData(double timeout)
    {
      T *update = 0;
      while (!update)
      {
        update = T::getUpdate(timeout);
        if (!update)
        {
          reconnect();
        }
      }
      return Ptr(update);
    }

    static void subscribe(double frequency)
    {
      T::subscribe(frequency);
    }

    static void unsubscribe()
    {
      T::subscribe(UNSUBSCRIBE);
    }

  };

} // namespace drobot_base
#endif  // DROBOT_BASE_LEGACY_WRAPPER_H
