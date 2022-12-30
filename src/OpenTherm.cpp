/*
OpenTherm.cpp - OpenTherm Communication Library For Arduino, ESP8266
Copyright 2018, Ihor Melnyk
*/

#include "OpenTherm.h"

OpenTherm::OpenTherm(int inPin, int outPin, bool isSlave):
	status(OpenThermStatus::NOT_INITIALIZED),
	inPin(inPin),
	outPin(outPin),
	isSlave(isSlave),
	response(0),
	responseStatus(OpenThermResponseStatus::NONE),
	responseTimestamp(0),
	handleInterruptCallback(NULL),
	processResponseCallback(NULL)
{
	timeout = 1000000;
}

void OpenTherm::begin(void(*handleInterruptCallback)(void), void(*processResponseCallback)(unsigned long, OpenThermResponseStatus, void*))
{
	pinMode(inPin, INPUT);
	pinMode(outPin, OUTPUT);
	if (handleInterruptCallback != NULL) {
		this->handleInterruptCallback = handleInterruptCallback;
		attachInterrupt(digitalPinToInterrupt(inPin), handleInterruptCallback, CHANGE);
	}
	activateBoiler();
	status = OpenThermStatus::READY;
	this->processResponseCallback = processResponseCallback;
}

void OpenTherm::begin(void(*handleInterruptCallback)(void))
{
	begin(handleInterruptCallback, NULL);
}

void OpenTherm::setTimeout(uint32_t timeout)
{
	this->timeout = timeout;
}
	
uint32_t OpenTherm::getTimeout()
{
	return timeout;
}

bool ICACHE_RAM_ATTR OpenTherm::isReady()
{
	return status == OpenThermStatus::READY;
}

int ICACHE_RAM_ATTR OpenTherm::readState() {
	return digitalRead(inPin);
}

void OpenTherm::setActiveState() {
	digitalWrite(outPin, LOW);
}

void OpenTherm::setIdleState() {
	digitalWrite(outPin, HIGH);
}

void OpenTherm::activateBoiler() {
	setIdleState();
	delay(1000);
}

void OpenTherm::sendBit(bool high) {
	if (high) setActiveState(); else setIdleState();
	delayMicroseconds(500);
	if (high) setIdleState(); else setActiveState();
	delayMicroseconds(500);
}

bool OpenTherm::sendRequestAync(unsigned long request)
{
	return sendRequestAync(request, nullptr);
}

bool OpenTherm::sendRequestAync(unsigned long request, void* userData)
{
	this->userData=userData;

	//Serial.println("Request: " + String(request, HEX));
	noInterrupts();
	const bool ready = isReady();
	interrupts();

	if (!ready)
	  return false;

	status = OpenThermStatus::REQUEST_SENDING;
	response = 0;
	responseStatus = OpenThermResponseStatus::NONE;

	sendBit(HIGH); //start bit
	for (int i = 31; i >= 0; i--) {
		sendBit(bitRead(request, i));
	}
	sendBit(HIGH); //stop bit
	setIdleState();

	status = OpenThermStatus::RESPONSE_WAITING;
	responseTimestamp = micros();
	return true;
}

unsigned long OpenTherm::sendRequest(unsigned long request)
{
	if (!sendRequestAync(request)) return 0;
	while (!isReady()) {
		process();
		yield();
	}
	return response;
}

bool OpenTherm::sendResponse(unsigned long request)
{
	status = OpenThermStatus::REQUEST_SENDING;
	response = 0;
	responseStatus = OpenThermResponseStatus::NONE;

	sendBit(HIGH); //start bit
	for (int i = 31; i >= 0; i--) {
		sendBit(bitRead(request, i));
	}
	sendBit(HIGH); //stop bit
	setIdleState();
	status = OpenThermStatus::READY;
	return true;
}

OpenThermResponseStatus OpenTherm::getLastResponseStatus()
{
	return responseStatus;
}

void ICACHE_RAM_ATTR OpenTherm::handleInterrupt()
{
	if (isReady())
	{
		if (isSlave && readState() == HIGH) {
		   status = OpenThermStatus::RESPONSE_WAITING;
		}
		else {
			return;
		}
	}

	unsigned long newTs = micros();
	if (status == OpenThermStatus::RESPONSE_WAITING) {
		if (readState() == HIGH) {
			status = OpenThermStatus::RESPONSE_START_BIT;
			responseTimestamp = newTs;
		}
		else {
			status = OpenThermStatus::RESPONSE_INVALID;
			responseTimestamp = newTs;
		}
	}
	else if (status == OpenThermStatus::RESPONSE_START_BIT) {
		if ((newTs - responseTimestamp < 750) && readState() == LOW) {
			status = OpenThermStatus::RESPONSE_RECEIVING;
			responseTimestamp = newTs;
			responseBitIndex = 0;
		}
		else {
			status = OpenThermStatus::RESPONSE_INVALID;
			responseTimestamp = newTs;
		}
	}
	else if (status == OpenThermStatus::RESPONSE_RECEIVING) {
		if ((newTs - responseTimestamp) > 750) {
			if (responseBitIndex < 32) {
				response = (response << 1) | !readState();
				responseTimestamp = newTs;
				responseBitIndex++;
			}
			else { //stop bit
				status = OpenThermStatus::RESPONSE_READY;
				responseTimestamp = newTs;
			}
		}
	}
}

void OpenTherm::process()
{
	noInterrupts();
	OpenThermStatus st = status;
	unsigned long ts = responseTimestamp;
	interrupts();

	if (st == OpenThermStatus::READY) return;

	unsigned long newTs = micros();
	if (st != OpenThermStatus::NOT_INITIALIZED && (newTs - ts) > timeout) {
		status = OpenThermStatus::READY;
		responseStatus = OpenThermResponseStatus::TIMEOUT;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus, userData);
		}
	}
	else if (st == OpenThermStatus::RESPONSE_INVALID) {
		status = OpenThermStatus::DELAY;
		responseStatus = OpenThermResponseStatus::INVALID;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus, userData);
		}
	}
	else if (st == OpenThermStatus::RESPONSE_READY) {
		status = OpenThermStatus::DELAY;
		responseStatus = (isSlave ? isValidRequest(response) : isValidResponse(response)) ? OpenThermResponseStatus::SUCCESS : OpenThermResponseStatus::INVALID;
		if (processResponseCallback != NULL) {
			processResponseCallback(response, responseStatus, userData);
		}
	}
	else if (st == OpenThermStatus::DELAY) {
		if ((newTs - ts) > 200000) {
			status = OpenThermStatus::READY;
		}
	}
}

bool OpenTherm::parity(unsigned long frame) //odd parity
{
	byte p = 0;
	while (frame > 0)
	{
		if (frame & 1) p++;
		frame = frame >> 1;
	}
	return (p & 1);
}

OpenThermMessageType OpenTherm::getMessageType(unsigned long message)
{
	OpenThermMessageType msg_type = static_cast<OpenThermMessageType>((message >> 28) & 7);
	return msg_type;
}

OpenThermMessageID OpenTherm::getDataID(unsigned long frame)
{
	return (OpenThermMessageID)((frame >> 16) & 0xFF);
}

unsigned long OpenTherm::buildRequest(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
	unsigned long request = data;
	if (type == OpenThermMessageType::WRITE_DATA) {
		request |= 1ul << 28;
	}
	request |= ((unsigned long)id) << 16;
	if (parity(request)) request |= (1ul << 31);
	return request;
}

unsigned long OpenTherm::buildResponse(OpenThermMessageType type, OpenThermMessageID id, unsigned int data)
{
	unsigned long response = data;
	response |= type << 28;
	response |= ((unsigned long)id) << 16;
	if (parity(response)) response |= (1ul << 31);
	return response;
}

bool OpenTherm::isValidResponse(unsigned long response)
{
	if (parity(response)) return false;
	byte msgType = (response << 1) >> 29;
	return msgType == READ_ACK || msgType == WRITE_ACK;
}

bool OpenTherm::isValidRequest(unsigned long request)
{
	if (parity(request)) return false;
	byte msgType = (request << 1) >> 29;
	return msgType == READ_DATA || msgType == WRITE_DATA;
}

void OpenTherm::end() {
	if (this->handleInterruptCallback != NULL) {
		detachInterrupt(digitalPinToInterrupt(inPin));
	}
}

const char *OpenTherm::statusToString(OpenThermResponseStatus status)
{
	switch (status) {
		case NONE:		return "NONE";
		case SUCCESS: 	return "SUCCESS";
		case INVALID: 	return "INVALID";
		case TIMEOUT: 	return "TIMEOUT";
		default:	  	return "UNKNOWN";
	}
}

const char *OpenTherm::messageTypeToString(OpenThermMessageType message_type)
{
	switch (message_type) {
		case READ_DATA:	   		return "READ_DATA";
		case WRITE_DATA:		return "WRITE_DATA";
		case INVALID_DATA:		return "INVALID_DATA";
		case RESERVED:			return "RESERVED";
		case READ_ACK:			return "READ_ACK";
		case WRITE_ACK:	   		return "WRITE_ACK";
		case DATA_INVALID:		return "DATA_INVALID";
		case UNKNOWN_DATA_ID:	return "UNKNOWN_DATA_ID";
		default:			  	return "UNKNOWN";
	}
}

const char *OpenTherm::messageIdToString(OpenThermMessageID messageID)
{
	switch (messageID)
	{
		case OpenThermMessageID::Status: 						return "Status";  
		case OpenThermMessageID::TSet:   						return "TSet";
		case OpenThermMessageID::MConfigMMemberIDcode:			return "MConfigMMemberIDcode"; 	//
		case OpenThermMessageID::SConfigSMemberIDcode:			return "SConfigSMemberIDcode"; //
		case OpenThermMessageID::Command:						return "Command";//
		case OpenThermMessageID::ASFflags:			  			return "ASFflags"; 
		case OpenThermMessageID::RBPflags:			  			return "RBPflags";
		case OpenThermMessageID::CoolingControl:	  			return "CoolingControl";
		case OpenThermMessageID::TsetCH2:			  			return "TsetCH2"; 
		case OpenThermMessageID::TrOverride:			  		return "TrOverride"; 
		case OpenThermMessageID::TSP:	  						return "TSP"; //
		case OpenThermMessageID::TSPindexTSPvalue:	  			return "TSPindexTSPvalue"; //
		case OpenThermMessageID::FHBsize:	  					return "FHBsize"; //
		case OpenThermMessageID::FHBindexFHBvalue:	  			return "FHBindexFHBvalue"; //
		case OpenThermMessageID::MaxRelModLevelSetting:	  		return "MaxRelModLevelSetting"; //
		case OpenThermMessageID::MaxCapacityMinModLevel:		return "MaxCapacityMinModLevel"; //
		case OpenThermMessageID::TrSet:	  						return "TrSet"; //
		case OpenThermMessageID::RelModLevel:					return "RelModLevel";
		case OpenThermMessageID::CHPressure:    				return "CHPressure";
		case OpenThermMessageID::DHWFlowRate:   				return "DHWFlowRate";
		case OpenThermMessageID::DayTime:	  					return "DayTime"; //
		case OpenThermMessageID::Date:	  						return "Date"; //
		case OpenThermMessageID::Year:	  						return "Year"; //
		case OpenThermMessageID::TrSetCH2:	  					return "TrSetCH2"; //
		case OpenThermMessageID::Tr:	  						return "Tr"; //
		case OpenThermMessageID::Tboiler: 						return "Tboiler";
		case OpenThermMessageID::Tdhw:							return "Tdhw";
		case OpenThermMessageID::Toutside:  					return "Toutside";
		case OpenThermMessageID::Tret:      					return "Tret";
		case OpenThermMessageID::Tstorage:  					return "Tstorage";
		case OpenThermMessageID::Tcollector:					return "Tcollector";
		case OpenThermMessageID::TflowCH2: 			 			return "TflowCH2";
		case OpenThermMessageID::Tdhw2:     					return "Tdhw2";       
		case OpenThermMessageID::Texhaust:      				return "Texhaust";
		case OpenThermMessageID::TdhwSetUBTdhwSetLB:			return "TdhwSetUBTdhwSetLB";
		case OpenThermMessageID::MaxTSetUBMaxTSetLB:    		return "MaxTSetUBMaxTSetLB";
		case OpenThermMessageID::HcratioUBHcratioLB:    		return "HcratioUBHcratioLB";
		case OpenThermMessageID::TdhwSet:			    		return "TdhwSet";
		case OpenThermMessageID::MaxTSet:			    		return "MaxTSet";
		case OpenThermMessageID::Hcratio:			    		return "Hcratio";
		case OpenThermMessageID::OEMDiagnosticCode:     		return "OEMDiagnosticCode";
		case OpenThermMessageID::BurnerStarts:		    		return "BurnerStarts";
		case OpenThermMessageID::CHPumpStarts:	        		return "CHPumpStarts";
		case OpenThermMessageID::DHWPumpValveStarts:    		return "DHWPumpValveStarts";
		case OpenThermMessageID::DHWBurnerStarts:       		return "DHWBurnerStarts";
		case OpenThermMessageID::BurnerOperationHours: 			return "BurnerOperationHours";
		case OpenThermMessageID::CHPumpOperationHours: 	 		return "CHPumpOperationHours";
		case OpenThermMessageID::DHWPumpValveOperationHours:	return "DHWPumpValveOperationHours";
		case OpenThermMessageID::DHWBurnerOperationHours: 	    return "DHWBurnerOperationHours";  
		case OpenThermMessageID::OpenThermVersionMaster: 	    return "OpenThermVersionMaster"; //
		case OpenThermMessageID::OpenThermVersionSlave: 	    return "OpenThermVersionSlave"; //
		case OpenThermMessageID::MasterVersion:			 	    return "MasterVersion"; //
		case OpenThermMessageID::SlaveVersion: 	    			return "SlaveVersion";  // 
		default:												return "UNKNOWN";
	}
}

const char *OpenTherm::messageToString(char *result, unsigned long message)
{
	int index = 0;
	strcpy(result + index, "TYPE=");
	index += sizeof("TYPE=") - 1;
	const char * tmp = messageTypeToString(getMessageType(message));	
	strcpy(result + index, tmp);
	index += strlen(tmp);

	strcpy(result + index, " ID=");
	index += sizeof(" ID=") - 1;
	tmp = messageIdToString(getDataID(message));	
	strcpy(result + index, tmp);
	index += strlen(tmp);
	
	strcpy(result + index, " DATA=");
	index += sizeof(" DATA=") - 1; 

	char buf[64];
	int i = 0;
	formatMessageData(buf, i, message);
	strcpy(result + index, buf);
	index += strlen(buf);

	strcpy(result + index, " MESSAGE=0x");
	index += sizeof(" MESSAGE=0x") - 1; 

  //result += String(message, HEX);
	memset(buf, 0x0, sizeof(buf));
	ultoa(message, buf, HEX);
	strcpy(result + index, buf);
	index += strlen(buf);
	
  return result;
}

void OpenTherm::formatMessageData(char *result, int& index, unsigned long message)
{
	char buf[16];

	OpenThermMessageID messageId = static_cast<OpenThermMessageID>(getDataID(message));

	if (isValueFloat(messageId))
		dtostrf(getFloat(message), 2 + 2, 2, buf);
	else if (isValueBits(messageId)){
		result[index++] = 'b';
		utoa(getUInt(message), buf, 2);	
	}
	else 
		switch (messageId){		
			case OpenThermMessageID::OEMDiagnosticCode:
			case OpenThermMessageID::BurnerStarts:
			case OpenThermMessageID::CHPumpStarts:
			case OpenThermMessageID::DHWPumpValveStarts:
			case OpenThermMessageID::DHWBurnerStarts:
			case OpenThermMessageID::BurnerOperationHours:
			case OpenThermMessageID::CHPumpOperationHours:
			case OpenThermMessageID::DHWPumpValveOperationHours:
			case OpenThermMessageID::DHWBurnerOperationHours:
				sprintf(buf, "%d", getUInt(message));	
			break;
			
			case OpenThermMessageID::TSP: 
			case OpenThermMessageID::TSPindexTSPvalue: 
			case OpenThermMessageID::FHBsize: 
			case OpenThermMessageID::FHBindexFHBvalue: 
			case OpenThermMessageID::MaxCapacityMinModLevel: 
			case OpenThermMessageID::TdhwSetUBTdhwSetLB:
			case OpenThermMessageID::MaxTSetUBMaxTSetLB:
			case OpenThermMessageID::HcratioUBHcratioLB:
			//	uint16_t v = getUInt(message);
			//	sprintf(buf, "%d/%d", v >> 8, v & 0xFF);
			//break;

			case OpenThermMessageID::Texhaust:
			default:
				result[index++] = '0';
				result[index++] = 'x';
				utoa(getUInt(message), buf, 16);	
			break;
		}
	
	strcpy(result + index, buf);
	index += strlen(buf);
	return;
}

bool OpenTherm::isValueFloat(OpenThermMessageID messageId)
{
	switch (messageId){
		case OpenThermMessageID::TSet:
		case OpenThermMessageID::TsetCH2: 
		case OpenThermMessageID::MaxRelModLevelSetting: 
		case OpenThermMessageID::TrSet: 
		case OpenThermMessageID::RelModLevel:
		case OpenThermMessageID::CHPressure:
		case OpenThermMessageID::DHWFlowRate:
		case OpenThermMessageID::TrSetCH2: 
		case OpenThermMessageID::Tr: 
		case OpenThermMessageID::Tboiler:  
		case OpenThermMessageID::Tdhw: // f8.8  DHW temperature (°C)
		case OpenThermMessageID::Toutside: // f8.8  Outside temperature (°C)
		case OpenThermMessageID::Tret: // f8.8  Return water temperature (°C)
		case OpenThermMessageID::Tstorage: // f8.8  Solar storage temperature (°C)
		case OpenThermMessageID::Tcollector: // f8.8  Solar collector temperature (°C)
		case OpenThermMessageID::TflowCH2: // f8.8  Flow water temperature CH2 circuit (°C)
		case OpenThermMessageID::Tdhw2: // f8.8  Domestic hot water temperature 2 (°C)   
		case OpenThermMessageID::TdhwSet:
		case OpenThermMessageID::MaxTSet:
		case OpenThermMessageID::Hcratio:   
			return true;

		default:
			return false;
	}
}

bool OpenTherm::isValueBits(OpenThermMessageID messageId)
{
	switch (messageId){
			case OpenThermMessageID::Status:
			case OpenThermMessageID::ASFflags:
			case OpenThermMessageID::RBPflags:
			return true;

		default:
			return false;
	}
}

//building requests

unsigned long OpenTherm::buildSetBoilerStatusRequest(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2) {
	unsigned int data = enableCentralHeating | (enableHotWater << 1) | (enableCooling << 2) | (enableOutsideTemperatureCompensation << 3) | (enableCentralHeating2 << 4);
	data <<= 8;
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Status, data);
}

unsigned long OpenTherm::buildSetBoilerTemperatureRequest(float temperature) {
	unsigned int data = temperatureToData(temperature);
	return buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TSet, data);
}

unsigned long OpenTherm::buildGetBoilerTemperatureRequest() {
	return buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tboiler, 0);
}

//parsing responses
bool OpenTherm::isFault(unsigned long response) {
	return response & 0x1;
}

bool OpenTherm::isCentralHeatingActive(unsigned long response) {
	return response & 0x2;
}

bool OpenTherm::isHotWaterActive(unsigned long response) {
	return response & 0x4;
}

bool OpenTherm::isFlameOn(unsigned long response) {
	return response & 0x8;
}

bool OpenTherm::isCoolingActive(unsigned long response) {
	return response & 0x10;
}

bool OpenTherm::isDiagnostic(unsigned long response) {
	return response & 0x40;
}

uint16_t OpenTherm::getUInt(const unsigned long response) const {
	const uint16_t u88 = response & 0xffff;
	return u88;
}

float OpenTherm::getFloat(const unsigned long response) const {
	const uint16_t u88 = getUInt(response);
	const float f = (u88 & 0x8000) ? -(0x10000L - u88) / 256.0f : u88 / 256.0f;
	return f;
}

unsigned int OpenTherm::temperatureToData(float temperature) {
	if (temperature < 0) temperature = 0;
	if (temperature > 100) temperature = 100;
	unsigned int data = (unsigned int)(temperature * 256);
	return data;
}

//basic requests

unsigned long OpenTherm::setBoilerStatus(bool enableCentralHeating, bool enableHotWater, bool enableCooling, bool enableOutsideTemperatureCompensation, bool enableCentralHeating2) {
	return sendRequest(buildSetBoilerStatusRequest(enableCentralHeating, enableHotWater, enableCooling, enableOutsideTemperatureCompensation, enableCentralHeating2));
}

bool OpenTherm::setBoilerTemperature(float temperature) {
	unsigned long response = sendRequest(buildSetBoilerTemperatureRequest(temperature));
	return isValidResponse(response);
}

float OpenTherm::getBoilerTemperature() {
	unsigned long response = sendRequest(buildGetBoilerTemperatureRequest());
	return isValidResponse(response) ? getFloat(response) : 0;
}

float OpenTherm::getReturnTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::Tret, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

bool OpenTherm::setDHWSetpoint(float temperature) {
    unsigned int data = temperatureToData(temperature);
    unsigned long response = sendRequest(buildRequest(OpenThermMessageType::WRITE_DATA, OpenThermMessageID::TdhwSet, data));
    return isValidResponse(response);
}
    
float OpenTherm::getDHWTemperature() {
    unsigned long response = sendRequest(buildRequest(OpenThermMessageType::READ_DATA, OpenThermMessageID::Tdhw, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

float OpenTherm::getModulation() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::RelModLevel, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

float OpenTherm::getPressure() {
    unsigned long response = sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::CHPressure, 0));
    return isValidResponse(response) ? getFloat(response) : 0;
}

unsigned char OpenTherm::getFault() {
    return ((sendRequest(buildRequest(OpenThermRequestType::READ, OpenThermMessageID::ASFflags, 0)) >> 8) & 0xff);
}
