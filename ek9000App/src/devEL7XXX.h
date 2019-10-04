/*

Driver for the EL70XX motor terminals
using the asyn API

*/
#ifndef _DEVEL7XXX_H_
#define _DEVEL7XXX_H_

#include <asynPortDriver.h>
#include <asynMotorAxis.h>
#include <asynMotorController.h>
#include <motor.h>
#include <drvModbusAsyn.h>
#include <modbus.h>
#include <modbusInterpose.h>

#include "devEK9000.h"

/* Axis class for the EL7XXXX */
class epicsShareClass EL7XXXAxis : public asynMotorAxis
{
public:
	EL7XXXAxis(class EL7XXXController* controller, int axis, int slavepos);

	/* Move the motor with the specified parameters */
	asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
	asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
	asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
	asynStatus stop(double acceleration);
	asynStatus poll(bool *moving);
	asynStatus setPosition(double position);
	asynStatus setClosedLoop(bool closedLoop);

	class EL7XXXController *pcontroller;
	/* Ptr to the bus coupler it uses */
	CEK9000Device *pcoupler;
	/* Ptr to the terminal info and such */
	CTerminal *pterminal;
};

/* Controller class for the EL7XXX */
class epicsShareClass EL7XXXController : public asynMotorController
{
public:
	EL7XXXController(const char* portName, CEK9000Device* device, int numAxes);
	//EL7XXXController(const char *portName, const char *portname, int numAxes, double movingPollPeriod, double idlePollPeriod);

	EL7XXXAxis *getAxis(asynUser *usr);

	EL7XXXAxis *getAxis(int axis_num);

	CEK9000Device* pcoupler;
};

#endif //_DEVEL7XXX_H_