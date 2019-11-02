#include <drvModbusAsyn.h>
#include <drvAsynIPPort.h>
#include <asynPortDriver.h>
#include <modbus.h>
#include <modbusInterpose.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <epicsString.h> 

/*

Driver for the ek9000, just handles queuing and other arbitration

*/

class drvEK9000 : public drvModbusAsyn
{
private:

public:
	drvEK9000(const char* port, const char* octetport);
	/*
	Perform a write to a terminal at the specified addr with the given function
	*/
	asynStatus writeTerminal(uint16_t addr, uint16_t fn, uint16_t* buf, int len);

};

drvEK9000::drvEK9000(const char* port, const char* octet) :
	drvModbusAsyn(port, octet, 0, 0, 0, 0, dataTypeInt16, 500, "")
{

}

asynStatus drvEK9000::writeTerminal(uint16_t addr, uint16_t fn, uint16_t* buf, int len)
{
	
	return asynSuccess;
}