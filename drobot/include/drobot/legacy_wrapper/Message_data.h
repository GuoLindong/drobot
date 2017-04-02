#ifndef MESSAGE_DATA_H
#define MESSAGE_DATA_H

#include <iostream>
#include <string>
#include <stdint.h>
#include "drobot/legacy_wrapper/Message.h"
#include "drobot/legacy_wrapper/Message_request.h"

namespace drobot
{

	class DataEncoders : public Message
	{
	public:
	  enum payloadOffsets
	  {
	    LEFT_FRONT_TRAVEL = 0,
	    LEFT_FRONT_SPEED = 4,
	    LEFT_FRONT_ANGLE = 6,
	    RIGHT_FRONT_TRAVEL = 8,
	    RIGHT_FRONT_SPEED = 12,
	    RIGHT_FRONT_ANGLE = 14,
	    LEFT_REAR_TRAVEL = 16,
	    LEFT_REAR_SPEED = 20,
	    LEFT_REAR_ANGLE = 22,
	    RIGHT_REAR_TRAVEL = 24,
	    RIGHT_REAR_SPEED = 28,
	    RIGHT_REAR_ANGLE = 30,
	    PAYLOAD_LEN = 32
	  };

	public:
	  DataEncoders(void *input, size_t msg_len);

	  DataEncoders(const DataEncoders &other);

	  static DataEncoders *popNext();

	  static DataEncoders *waitNext(double timeout = 0);

	  static DataEncoders *getUpdate(double timeout = 0);

	  static void subscribe(uint16_t freq);

	  static enum MessageTypes getTypeID();

	  int32_t getLeftFrontTravel();
	  
	  double getLeftFrontSpeed();

	  double getLeftFrontAngle();

	  int32_t getRightFrontTravel();

	  double getRightFrontSpeed();

	  double getRightFrontAngle();

	  int32_t getLeftRearTravel();
	  
	  double getLeftRearSpeed();

	  double getLeftRearAngle();

	  int32_t getRightRearTravel();

	  double getRightRearSpeed();

	  double getRightRearAngle();

	  virtual std::ostream &printMessage(std::ostream &stream = std::cout);
	};

	class DataSystemStatus : public Message
	{
	public:
	enum payloadOffsets
	{
	  VOLTAGE= 0,
	  TEMPERATURE = 2,
	  LF_DRIVE_PWM = 4,
	  LF_TURN_PWM,
	  LF_ERROR,
	  RF_DRIVE_PWM,
	  RF_TURN_PWM,
	  RF_ERROR,
	  LR_DRIVE_PWM,
	  LR_TURN_PWM,
	  LR_ERROR,
	  RR_DRIVE_PWM,
	  RR_TURN_PWM,
	  RR_ERROR,
	  PAYLOAD_LEN = 16
	};

	public:
	  DataSystemStatus(void *input, size_t msg_len);

	  DataSystemStatus(const DataSystemStatus &other);

	  static DataSystemStatus *popNext();

	  static DataSystemStatus *waitNext(double timeout = 0);

	  static DataSystemStatus *getUpdate(double timeout = 0);

	  static void subscribe(uint16_t freq);

	  static enum MessageTypes getTypeID();

	  double getVoltage();

	  double getTemperature();

	  int8_t getLfDrivePwm();
	  int8_t getLfTurnPwm();
	  int8_t getLfError();

	  int8_t getRfDrivePwm();
	  int8_t getRfTurnPwm();
	  int8_t getRfError();

	  int8_t getLrDrivePwm();
	  int8_t getLrTurnPwm();
	  int8_t getLrError();

	  int8_t getRrDrivePwm();
	  int8_t getRrTurnPwm();
	  int8_t getRrError();

	  virtual std::ostream &printMessage(std::ostream &stream = std::cout);
	};

}; // namespace drobot

#endif // MESSAGE_DATA_H