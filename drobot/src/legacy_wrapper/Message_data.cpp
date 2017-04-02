#include "drobot/legacy_wrapper/Message_data.h"
#include "drobot/legacy_wrapper/Number.h"
#include "drobot/legacy_wrapper/Transport.h"

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <ros/ros.h>

using namespace std;

namespace drobot
{
	/**
	* Macro which generates definitions of the Message constructors
	* ExpectedLength is an expression valid within the constructor which gives the
	* expected payload length of the message.  If the length reported in the message
	* header does not match, and exception will be thrown.  If ExpectedLength is -1,
	* the length check will be skipped.
	* NB: Some Messages need to do some extra work in the constructor and don't use
	*     this macro!
	*/
	#define MESSAGE_CONSTRUCTORS(MessageClass, ExpectedLength) \
	MessageClass::MessageClass(void* input, size_t msg_len) : Message(input, msg_len) \
	{ \
	    if( ((ExpectedLength) >= 0) && ((ssize_t)getPayloadLength() != (ExpectedLength)) ) { \
	        stringstream ss; \
	        ss << "Bad payload length: actual="<<getPayloadLength(); \
	        ss <<" vs. expected="<<(ExpectedLength); \
	        throw new MessageException(ss.str().c_str(), MessageException::INVALID_LENGTH); \
	    } \
	} \
	MessageClass::MessageClass(const MessageClass& other) : Message(other) {}

	/**
	* Macro which generates definitios of the Message convenience functions
	* All message classes should use this macro to define these functions.
	*/
	#define MESSAGE_CONVENIENCE_FNS(MessageClass, DataMsgID) \
	MessageClass* MessageClass::popNext() { \
	    return dynamic_cast<MessageClass*>(Transport::instance().popNext(DataMsgID)); \
	} \
	\
	MessageClass* MessageClass::waitNext(double timeout) { \
	    return dynamic_cast<MessageClass*>(Transport::instance().waitNext(DataMsgID, timeout)); \
	} \
	\
	MessageClass* MessageClass::getUpdate(double timeout) { \
	    Transport::instance().flush(DataMsgID); \
	    subscribe(0); \
	    return dynamic_cast<MessageClass*>( \
	            Transport::instance().waitNext(DataMsgID, timeout) ); \
	}\
	\
	void MessageClass::subscribe(uint16_t freq) { \
	    Request(DataMsgID-0x0040, freq).send(); \
	} \
	\
	enum MessageTypes MessageClass::getTypeID() { \
	    return DataMsgID; \
	}


	DataEncoders::DataEncoders(void *input, size_t msg_len) : Message(input, msg_len)
	{	  
	  if ((ssize_t) getPayloadLength() != (4 * 8))
	  {
	    stringstream ss;
	    ss << "Bad payload length: actual=" << getPayloadLength();
	    ss << " vs. expected=" << (4 * 8);
	    throw new MessageException(ss.str().c_str(), MessageException::INVALID_LENGTH);
	  }
	}

	DataEncoders::DataEncoders(const DataEncoders &other) : Message(other)
	{
	}

	MESSAGE_CONVENIENCE_FNS(DataEncoders, DATA_ENCODER);

	int32_t DataEncoders::getLeftFrontTravel()
	{
		
		return btoi(getPayloadPointer(LEFT_FRONT_TRAVEL), 4);
	}
	  
	double DataEncoders::getLeftFrontSpeed()
	{
		return btof(getPayloadPointer(LEFT_FRONT_SPEED), 2, 100);
	}

	double DataEncoders::getLeftFrontAngle()
	{
		return btof(getPayloadPointer(LEFT_FRONT_ANGLE), 2, 10);
	}

	int32_t DataEncoders::getRightFrontTravel()
	{
		return btoi(getPayloadPointer(RIGHT_FRONT_TRAVEL), 4);
	}

	double DataEncoders::getRightFrontSpeed()
	{
		return btof(getPayloadPointer(RIGHT_FRONT_SPEED), 2, 100);
	}

	double DataEncoders::getRightFrontAngle()
	{
		return btof(getPayloadPointer(RIGHT_FRONT_ANGLE), 2, 10);
	}

	int32_t DataEncoders::getLeftRearTravel()
	{
		return btoi(getPayloadPointer(LEFT_REAR_TRAVEL), 4);
	}
	  
	double DataEncoders::getLeftRearSpeed()
	{
		return btof(getPayloadPointer(LEFT_REAR_SPEED), 2, 100);
	}

	double DataEncoders::getLeftRearAngle()
	{
		return btof(getPayloadPointer(LEFT_REAR_ANGLE), 2, 10);
	}

	int32_t DataEncoders::getRightRearTravel()
	{
		return btoi(getPayloadPointer(RIGHT_REAR_TRAVEL), 4);
	}

	double DataEncoders::getRightRearSpeed()
	{
		return btof(getPayloadPointer(RIGHT_REAR_SPEED), 2, 100);
	}

	double DataEncoders::getRightRearAngle()
	{
		return btof(getPayloadPointer(RIGHT_REAR_ANGLE), 2, 10);
	}

	ostream &DataEncoders::printMessage(ostream &stream)
	{
	  stream << "Encoder Data" << endl;
	  stream << "============" << endl;
	  stream << "  Travel: " << getLeftFrontTravel() << endl;
	  stream << "  Speed : " << getLeftFrontSpeed()  << endl;
	  return stream;
	}

	DataSystemStatus::DataSystemStatus(void *input, size_t msg_len) : Message(input, msg_len)
	{	  

	  if ((ssize_t) getPayloadLength() != (16))
	  {
	    stringstream ss;
	    ss << "Bad payload length: actual=" << getPayloadLength();
	    ss << " vs. expected=" << (16);
	    throw new MessageException(ss.str().c_str(), MessageException::INVALID_LENGTH);
	  }
	}

	DataSystemStatus::DataSystemStatus(const DataSystemStatus &other) : Message(other)
	{
	}

	MESSAGE_CONVENIENCE_FNS(DataSystemStatus, DATA_SYSTEM_STATUS);

	double DataSystemStatus::getVoltage()
	{
		return btof(getPayloadPointer(VOLTAGE), 2, 100);
	}

	double DataSystemStatus::getTemperature()
	{
		return btof(getPayloadPointer(TEMPERATURE), 2, 100);
	}

	int8_t DataSystemStatus::getLfDrivePwm()
	{
		return btou(getPayloadPointer(LF_DRIVE_PWM), 1);
	}
	  
	int8_t DataSystemStatus::getLfTurnPwm()
	{
		return btou(getPayloadPointer(LF_TURN_PWM), 1);
	}

	int8_t DataSystemStatus::getLfError()
	{
		return btou(getPayloadPointer(LF_ERROR), 1);
	}

	int8_t DataSystemStatus::getRfDrivePwm()
	{
		return btou(getPayloadPointer(RF_DRIVE_PWM), 1);
	}

	int8_t DataSystemStatus::getRfTurnPwm()
	{
		return btou(getPayloadPointer(RF_TURN_PWM), 1);
	}

	int8_t DataSystemStatus::getRfError()
	{
		return btou(getPayloadPointer(RF_ERROR), 1);
	}

	int8_t DataSystemStatus::getLrDrivePwm()
	{
		return btou(getPayloadPointer(LR_DRIVE_PWM), 1);
	}

	int8_t DataSystemStatus::getLrTurnPwm()
	{
		return btou(getPayloadPointer(LR_TURN_PWM), 1);
	}

	int8_t DataSystemStatus::getLrError()
	{
		return btou(getPayloadPointer(LR_ERROR), 1);
	}

	int8_t DataSystemStatus::getRrDrivePwm()
	{
		return btou(getPayloadPointer(RR_DRIVE_PWM), 1);
	}

	int8_t DataSystemStatus::getRrTurnPwm()
	{
		return btou(getPayloadPointer(RR_TURN_PWM), 1);
	}

	int8_t DataSystemStatus::getRrError()
	{
		return btou(getPayloadPointer(RR_ERROR), 1);
	}

	ostream &DataSystemStatus::printMessage(ostream &stream)
	{
	  stream << "Encoder Data" << endl;
	  stream << "============" << endl;
	  stream << "  Voltage: " << getVoltage() << endl;
	  stream << "  Temperature : " << getTemperature()  << endl;
	  return stream;
	}
	
}; //namespace drobot