//======================================================//
// Name: devEL7XXX.cpp
// Purpose: Device support for EL7xxx modules (motor control)
//          requires the motor record module for epics
// Authors: Jeremy L.
// Date Created: July 17, 2019
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
#include <boRecord.h>
#include <iocsh.h>
#include <callback.h>

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <drvAsynIPPort.h>

/* Motor record */
#include <motor.h>
#include <motorRecord.h>
#include <motordrvCom.h>
#include <motordevCom.h>
#include <motor_interface.h>
#include <motordrvComCode.h>
#include <asynMotorAxis.h>
#include <asynMotorController.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <vector>

#include "asynDriver.h"
#include "devEK9000.h"
#include "devEL7XXX.h"
#include "errlog.h"

#define BREAK() /* asm("int3\n\t") */

std::vector<el70x7Controller*> controllers;

/*

Class for modbus driver asyn users

*/

/*
 * RAII lock
 */ 
el70x7Axis::CouplerLock::CouplerLock(el70x7Axis* axis)
{
	this->axis = axis;
	int stat = this->axis->pC_->pcoupler->Lock();
	if (stat) epicsAssert(__FILE__, __LINE__, "pC_->lock() != asynSuccess", "");
	asynPrint(axis->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Grabbed mutex lock for coupler=%s port=%s axisno=%u\n",
		__FUNCTION__, __LINE__, axis->pcoupler->m_pName,
		axis->pcoupler->m_pPortName, axis->axisNo_);
}

el70x7Axis::CouplerLock::~CouplerLock()
{
	this->axis->pC_->pcoupler->Unlock();
	asynPrint(axis->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Released mutex for coupler=%s port=%s axisno=%u\n",
		__FUNCTION__, __LINE__, axis->pcoupler->m_pName,
		axis->pcoupler->m_pPortName, axis->axisNo_);
}

/*
========================================================

class EL70X7Controller

NOTES:
	- None

========================================================
*/

el70x7Controller::el70x7Controller(CEK9000Device* dev, CTerminal* controller, const char* port, int numAxis) :
	asynMotorController(port, numAxis, 0, 0, 0, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0)
{
	pcoupler = dev;
	pcontroller = controller;
	this->paxis = (el70x7Axis**)calloc(sizeof(el70x7Axis*), numAxis);
	for(int i = 0; i < numAxis; i++)
		this->paxis[i] = new el70x7Axis(this, i);
	startPoller(0.25, 0.25, 0);
	if(!dev->VerifyConnection())
		asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Unable to connect to device.\n");
}

el70x7Axis* el70x7Controller::getAxis(int num)
{
	return (el70x7Axis*)asynMotorController::getAxis(num);
}

el70x7Axis* el70x7Controller::getAxis(asynUser* usr)
{
	return (el70x7Axis*)asynMotorController::getAxis(usr);
}

void el70x7Controller::report(FILE* fd, int lvl)
{
	if(lvl)
		fprintf(fd, "\tel70x7Controller slave=%u\n", this->pcontroller->m_nTerminalIndex);
	asynMotorController::report(fd, lvl);
}

/*
========================================================

class EL70X7Axis

NOTES:
	- The EL7047 takes acceleration in ms which represents
	the time to top speed
	- Speed is set when the class is created because it shouldn't
	change during normal operation. I do not want to be doing
	a ton of CoE I/O when we're doing semi-time critical ops
	- This class is the logical representation of a single
	el7047 or el7037

========================================================
*/


el70x7Axis::el70x7Axis(el70x7Controller* pC, int axisnum) :
	asynMotorAxis(pC, axisnum)
{
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u\n",
			__FUNCTION__, __LINE__);
	uint16_t spd;
	pC_= pC;
	this->pcoupler = pC->pcoupler;
	this->pcontroller = pC->pcontroller;
	this->pdrv = pC->pcoupler->m_pDriver;
	this->lock();
	/* Set previous params to random values */
	/* Grab initial values */
	int status = this->pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 
		pcontroller->m_nOutputStart, (uint16_t*)&this->output, pcontroller->m_nOutputSize/2);
	if(status) goto error;
	status = this->pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS,
		pcontroller->m_nInputStart, (uint16_t*)&this->input, pcontroller->m_nInputSize/2);
	/* Read the configured speed */
	spd = 0;
	this->pcoupler->doCoEIO(0, pcontroller->m_nTerminalIndex, 0x8012, 1, &spd, 0x05);
	/* THIS IS IMPORTANT: Set everything to zero initially */
	this->curr_param.back_accel = 0.0;
	this->curr_param.forward_accel = 0.0;
	this->curr_param.max_vel = 0.0;
	this->curr_param.min_vel = 0.0;
	/* ALSO set these to zero or else it will be full of junk values that'll get possibly written to the device if there is an input error */
	memset(&this->input, 0, sizeof(SPositionInterfaceCompact_Input));
	memset(&this->output, 0, sizeof(SPositionInterfaceCompact_Output));
	this->output.stm_enable = 1; /* Enable the motor */
	switch(spd)
	{
		case 0:		speed = 1000;	break;
		case 1:		speed = 2000;	break;
		case 2:		speed = 4000;	break;
		case 3:		speed = 8000;	break;
		case 4:		speed = 16000;	break;
		case 5:		speed = 32000;	break;
		default:	speed = 1000; 	break;
	}
	this->unlock();
	return;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Modbus IO error. Error=%u\n",
		__FUNCTION__, __LINE__, status);
	this->unlock();
}

void el70x7Axis::lock()
{
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u\n",
		__FUNCTION__, __LINE__);
	this->pcoupler->Lock();
}

void el70x7Axis::unlock()
{
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u\n",
		__FUNCTION__, __LINE__);
	this->pcoupler->Unlock();
}

asynStatus el70x7Axis::setMotorParameters(uint16_t min_start_vel,
		uint16_t max_coil_current, uint16_t reduced_coil_currrent, uint16_t nominal_voltage,
		uint16_t internal_resistance, uint16_t full_steps, uint16_t enc_inc)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u\n",
		__FUNCTION__, __LINE__);
	uint16_t tid = pcontroller->m_nTerminalIndex;
	int stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &max_coil_current, 0x1);
	if(stat) goto error;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &reduced_coil_currrent, 0x2);
	if(stat) goto error;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &nominal_voltage, 0x3);
	if(stat) goto error;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &internal_resistance, 0x4);
	if(stat) goto error;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &full_steps, 0x6);
	if(stat) goto error;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &enc_inc, 0x7);
	if(stat) goto error;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &min_start_vel, 0x9);
	if(stat) goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to propagate CoE params error=%u\n",
		__FUNCTION__, __LINE__, stat);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::move(double pos, int rel, double min_vel, double max_vel, double accel)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::move\n",
		__FUNCTION__, __LINE__);
	/* Set the params */
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	this->curr_param.forward_accel = accel;
	int stat = 0;//UpdateParams();
	if(stat) goto error;
	if(rel)
		output.pos_tgt_pos = (uint32_t)(input.cntr_val + pos);
	else
		output.pos_tgt_pos = ((uint32_t)pos);
	output.pos_execute = 1; /* set to 1 to set counter */
	output.pos_emergency_stp = 0;
	/* Execute move */
	stat = Execute();
	if(stat) goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to perform move.\n",
		__FUNCTION__, __LINE__);
	this->unlock();
	return asynError;
}

/*
Move the motor at a constant velocity until the stop signal is recieved.
min_vel is the starting velocity of the motor, max_vel is the max velocity of the motor
the velocity is in steps/s
accel is the acceleration of the motor in steps/s^2
*/
asynStatus el70x7Axis::moveVelocity(double min_vel, double max_vel, double accel)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::moveVelocity\n",
		__FUNCTION__,__LINE__);
	/* Update relevant parameters */
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	this->curr_param.forward_accel = accel;
	int stat = this->UpdateParams();
	if(stat) goto error;
	/* Set velocity params */
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to set move velocity.\n",
		__FUNCTION__, __LINE__);
	this->unlock();
	return asynError;
}

/*
Move the motor to it's home position
*/
asynStatus el70x7Axis::home(double min_vel, double max_vel, double accel, int forwards)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::home\n",
		__FUNCTION__,__LINE__);
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	if(forwards)
		this->curr_param.forward_accel = accel;
	else
		this->curr_param.back_accel = accel;
	int stat = this->UpdateParams();
	if(stat) goto error;
	/* Home is just going to be 0 for now */
	output.pos_tgt_pos = 0;
	output.pos_emergency_stp = 0;
	stat = Execute();
	if(stat) goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to go to home position.\n",
		__FUNCTION__,__LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::stop(double accel)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::stop\n",
		__FUNCTION__, __LINE__);
	/* TODO: Investigate if we have the time required to write a new accel value to the terminal */
	this->curr_param.back_accel = accel;
	int stat = this->UpdateParams();
	if(stat) goto error;
	/* TODO: We should not be using emergency stop here. */
	output.pos_emergency_stp = 1;
	output.pos_execute = 0;
	this->UpdatePDO();
	if(stat) goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to stop motor.\n",
		__FUNCTION__,__LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::poll(bool* moving)
{
	if(!this->pcoupler->VerifyConnection())
	{
		asynPrint(this->pasynUser_, ASYN_TRACE_WARNING, "%s:%u Polling skipped because device is not connected.\n",
			__FUNCTION__, __LINE__);
		return asynSuccess;
	}
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::poll\n",
		__FUNCTION__,__LINE__);
	/* This will read params from the motor controller */
	int stat = this->UpdatePDO();
	if(stat) goto error;
	/* encoderposition is double */
	this->setDoubleParam(pC_->motorEncoderPosition_, (double)input.cntr_val);
	this->setIntegerParam(pC_->motorStatusDone_, input.stat_in_tgt);
	this->setIntegerParam(pC_->motorStatusDirection_, input.stm_mov_pos);
	this->setIntegerParam(pC_->motorStatusSlip_, input.stm_stall);
	this->setIntegerParam(pC_->motorStatusProblem_, input.stm_err);
	/* Check for counter overflow or underflow */
	if(input.cntr_overflow || input.cntr_underflow)
		asynPrint(this->pasynUser_, ASYN_TRACE_WARNING, "%s: Stepper motor counter overflow/underflow detected.\n",
			__FUNCTION__);
	/* Checo for other error condition */
	if(input.stm_err)
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s: Stepper motor error detected.\n",
			__FUNCTION__);
	*moving = input.stat_busy != 0;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to poll device.\n",
		__FUNCTION__,__LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::setPosition(double pos)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::setPosition val=%f\n",
		__FUNCTION__, __LINE__, pos);
	output.pos_tgt_pos = (uint32_t)round(pos);
	output.pos_execute = 1;
	output.pos_emergency_stp = 0;
	output.stm_enable = 1;
	int stat = UpdatePDO();
	if(stat) goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Error while setting tgt pos.\n",
		__FUNCTION__, __LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::setEncoderPosition(double pos)
{
	this->lock();
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::setEncoderPosition",
		__FUNCTION__, __LINE__);
	output.enc_counter_val = (uint32_t)round(pos);
	output.enc_set_counter = 1;
	int stat = UpdatePDO();
	if(stat) goto error;
	this->unlock();
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Error while setting encoder position.\n",
		__FUNCTION__, __LINE__);
	this->unlock();
	return asynError;
}

asynStatus el70x7Axis::setClosedLoop(bool closed) 
{
	return asynSuccess;
}

void Int32Fix(uint32_t* v)
{
	uint32_t tmp = *v;
	((uint16_t*)v)[0] = ((uint16_t*)&tmp)[1];
	((uint16_t*)v)[1] = ((uint16_t*)&tmp)[0];
}

void FixEndian(SPositionInterfaceCompact_Input& in, SPositionInterfaceCompact_Output& out)
{
	Int32Fix(&in.cntr_val);
	Int32Fix(&in.lat_val);
	Int32Fix(&out.enc_counter_val);
	Int32Fix(&out.pos_tgt_pos);
}

asynStatus el70x7Axis::UpdatePDO(bool locked)
{
	const char* pStep = "UPDATE_PDO";
	SPositionInterfaceCompact_Input old_input = this->input;
	/* Update input pdos */
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n",
		__FUNCTION__, __LINE__, pStep);
	int stat = pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, pcontroller->m_nInputStart,
		(uint16_t*)&this->input, 7);
	if(stat) goto error;
	pStep = "PROPAGATE_PDO";
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n",
		__FUNCTION__, __LINE__, pStep);
	/* Propagate changes from our internal pdo */
	stat = pcoupler->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_nOutputStart,
		(uint16_t*)&this->output, 7);
	if(stat) goto error;
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Error while updating PDOs. Error=%u Step=%s Input Start=%u"
			" Input size=%u Output Start=%u Output Size=%u\n",
		__FUNCTION__, __LINE__, stat, pStep, pcontroller->m_nInputStart, pcontroller->m_nInputSize/2,
		pcontroller->m_nOutputStart, pcontroller->m_nOutputSize/2);
	/* Reset to previous on error */
	this->input = old_input;
	//this->output = old_output;
	return asynError;
}

/*
Executes a move by setting the execute bit and propagating changes
*/
asynStatus el70x7Axis::Execute(bool locked)
{
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::Execute\n",
		__FUNCTION__, __LINE__);
	this->output.pos_execute = 1;
	this->output.pos_emergency_stp = 0;
	int stat = this->UpdatePDO();
	if(stat) goto error;
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to execute moves.\n",
		__FUNCTION__, __LINE__);
	return asynError;
}

void el70x7Axis::report(FILE* fd, int lvl)
{
	if(lvl)
		fprintf(fd, "\t\tel70x7Axis.\n");
	asynMotorAxis::report(fd, lvl);
}

/*
Update the specified parameters
*/
asynStatus el70x7Axis::UpdateParams()
{
	asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u el70x7Axis::UpdateParams\n",
		__FUNCTION__, __LINE__);
	int res = 0;
	/* Check which params have changed and then propagate changes */
	const char* step = "";
	if(curr_param.forward_accel != prev_param.forward_accel)
	{
		step = "Update_Forward_Accel";
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n",
			__FUNCTION__, __LINE__, step);
		/* the 7047 takes accel in the units of ms, but the motor record gives it to us in steps/s^2 */
		uint32_t data = (uint32_t)((curr_param.max_vel-curr_param.min_vel)/curr_param.forward_accel);
		res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 3);
		if(res) goto error;
		prev_param.forward_accel = curr_param.forward_accel;
	}
	if(curr_param.back_accel != prev_param.back_accel)
	{
		step = "Update_Back_Accel";
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n",
			__FUNCTION__, __LINE__, step);
		/* same as forward accel */
		uint32_t data = (uint32_t)((curr_param.max_vel-curr_param.min_vel)/curr_param.back_accel);
		res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 4);
		if(res) goto error;
		prev_param.back_accel = curr_param.back_accel;
	}
	if(curr_param.max_vel != prev_param.max_vel)
	{
		step = "Update_Max_Vel";
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n",
			__FUNCTION__, __LINE__, step);
		/* The speed is set on init of this axis, and it should not change during operation */
		/* The el7047 takes speed in terms of the percentage of the max velocity, multiplied by 10,000 (it likes integers) */
		uint32_t data = (uint32_t)((curr_param.max_vel/(double)this->speed)*10000);
		res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 2);
		if(res) goto error;
		prev_param.max_vel = curr_param.max_vel;
	}
	if(curr_param.min_vel != prev_param.min_vel)
	{
		step = "Update_Min_Vel";
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Step: %s\n",
			__FUNCTION__, __LINE__, step);
		uint32_t data = (uint32_t)((curr_param.min_vel/(double)this->speed)*10000);
		res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 1);
		if(res) goto error;
		/* Need to propagate to STM Settings Part 1, 0x9: starting velocity */
		//res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8010, 2, (uint16_t*)&data, 0x9);
		prev_param.min_vel = curr_param.min_vel;
	}
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to update params. Error=%u Step=%s\n",
		__FUNCTION__, __LINE__, res, step);
	return asynError;
}


/*
 * ek7047Configure(string ek9k, )
 */ 
void el7047_Configure(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	const char* port = args[1].sval;
	const char* rec = args[2].sval;
	int slaveid = args[3].ival;
	if(!ek9k)
	{
		epicsPrintf("Please provide an ek9000 name.\n");
		return;
	}
	if(!port || !rec)
	{
		epicsPrintf("Please provide a port name.\n");
		return;
	}
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);

	if(!dev)
	{
		epicsPrintf("Device not found.\n");
		return;
	}
	CTerminal* term = &dev->m_pTerms[slaveid-1];
	dev->AddTerminal(rec, 7047, slaveid);
	term->m_nInputSize = 14;
	term->m_nOutputSize = 14;
	el70x7Controller* pctl = new el70x7Controller(dev, term, port, 1);
	printf("Created motor port %s\n", port);
	controllers.push_back(pctl);
}

void el7047_Stat(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	if(!ek9k)
	{
		epicsPrintf("Please provide an ek9000 name.\n");
		return;
	}
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);
	if(!dev)
	{
		epicsPrintf("Invalid device.\n");
		return;
	}
	for(auto x : controllers)
	{
		for(int i = 0; ; i++)
		{
			el70x7Axis* axis = x->getAxis(i);
			if(!axis) break;
			epicsPrintf("%s\n", x->pcontroller->m_pRecordName);
			epicsPrintf("\tSpeed [steps/s]:      %u\n", axis->speed);
			epicsPrintf("\tEncoder pos:          %u\n", axis->enc_pos);
		}
	}
}

/* Configuration param support */
static coe_param_t coe_params[] = {
	{"maximal-current",  "mA",          0x8010, 0x1, coe_param_t::COE_PARAM_INT16, -1},
	{"reduced-current",  "mA",          0x8010, 0x2, coe_param_t::COE_PARAM_INT16, -1},
	{"nominal-voltage",  "10mV",        0x8010, 0x3, coe_param_t::COE_PARAM_INT16, -1},
	{"coil-resistance",  "10mOhm",      0x8010, 0x4, coe_param_t::COE_PARAM_INT16, -1},
	{"motor-emf",        "1mv/(rad/s)", 0x8010, 0x5, coe_param_t::COE_PARAM_INT16, -1},
	{"motor-fullsteps",  "steps",       0x8010, 0x6, coe_param_t::COE_PARAM_INT16, -1},
	{"motor-inductance", "0.01mH",      0x8010, 0xA, coe_param_t::COE_PARAM_INT16, -1},
	{"target-window",    "no unit",     0x8020, 0xB, coe_param_t::COE_PARAM_INT16, -1},
	{"velocity-max",     "steps/s",     0x8020, 0x2, coe_param_t::COE_PARAM_INT16, -1},
	{"velocity-min",     "steps/s",     0x8020, 0x1, coe_param_t::COE_PARAM_INT16, -1},
	{} /* Needed to terminate the list */
};


/* Read from CoE object */
void el70x7ReadCoE(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	const char* port = args[1].sval;
	int index = args[2].ival;
	int subindex = args[3].ival;
	int len = args[4].ival;

	if(!ek9k || !port || index < 0 || index > 65535 || subindex < 0 || subindex > 65535 || len < 0)
		return;

	for(el70x7Controller* x : controllers)
	{
		if(strcmp(port, x->portName) == 0)
		{
			x->pcoupler->Lock();
			uint16_t data[32];
			x->pcoupler->doCoEIO(0, x->pcontroller->m_nTerminalIndex, 
				index, len, data, subindex);
			for(int j = 0; j < len; j++)
				epicsPrintf("%u ", data[j]);
			epicsPrintf("\n");			
			x->pcoupler->Unlock();
			return;
		}
	}
	epicsPrintf("Port not found.\n");
}

void el70x7GetParam(const iocshArgBuf* args)
{
	const char* port = args[0].sval;
	const char* param = args[1].sval;
	if(!port || !param)
	{
		for(unsigned i = 0; i < sizeof(coe_params) / sizeof(coe_param_t); i++)
			epicsPrintf("\t%s [%s]\n", coe_params[i].name, coe_params[i].unit);
		return;
	}
	for(el70x7Controller* x : controllers)
	{
		if(strcmp(port, x->portName) == 0)
		{
			x->pcoupler->Lock();

			for(unsigned i = 0; i < sizeof(coe_params)/sizeof(coe_param_t); i++)
			{
				coe_param_t cparam = coe_params[i];
				if(strcmp(param, cparam.name) == 0)
				{
					uint16_t tmpval;
					uint16_t tmpval32[2];
					int status=0;
					switch(coe_params[i].type)
					{
						case coe_param_t::COE_PARAM_INT16:
							tmpval = 0;
							status = x->pcoupler->doCoEIO(0,
								x->pcontroller->m_nTerminalIndex, cparam.index, 1, &tmpval,
								cparam.subindex);
								epicsPrintf("%s: %u\n", cparam.name, tmpval);
							if(status)
								epicsPrintf("Could not get %s \n", cparam.name);
							goto end;
						case coe_param_t::COE_PARAM_INT32:
							tmpval32[0] = 0;
							status = x->pcoupler->doCoEIO(0,
								x->pcontroller->m_nTerminalIndex, cparam.index, 4, tmpval32,
								cparam.subindex);
								epicsPrintf("%s: %u\n", cparam.name, tmpval32[0]);
							if(status)
								epicsPrintf("Could not get %s\n", cparam.name);
							goto end;
						case coe_param_t::COE_PARAM_STRING:
						case coe_param_t::COE_PARAM_FLOAT32:
						default: goto end;
					}
				}
			}
		end:
			x->pcoupler->Unlock();
			return;
		}
	}
	epicsPrintf("Port not found.\n");
}

void el70x7SetParam(const iocshArgBuf* args)
{
	const char* port = args[0].sval;
	const char* param = args[1].sval;
	if(!port || !param)
	{
		for(unsigned i = 0; i < sizeof(coe_params) / sizeof(coe_param_t); i++)
			epicsPrintf("\t%s [%s]\n", coe_params[i].name, coe_params[i].unit);
		return;
	}
	for(el70x7Controller* x : controllers)
	{
		if(strcmp(port, x->portName) == 0)
		{
			x->pcoupler->Lock();

			for(unsigned i = 0; i < sizeof(coe_params)/sizeof(coe_param_t); i++)
			{
				coe_param_t cparam = coe_params[i];
				if(strcmp(param, cparam.name) == 0)
				{
					uint16_t tmpval;
					uint16_t tmpval32[2];
					int status;
					switch(coe_params[i].type)
					{
						case coe_param_t::COE_PARAM_INT16:
							tmpval = args[3].ival;
							status = x->pcoupler->doCoEIO(1,
								x->pcontroller->m_nTerminalIndex, cparam.index, 1, &tmpval,
								cparam.subindex);
							if(status)
								epicsPrintf("Could not set %s to %u\n", cparam.name, tmpval);
							goto end;
						case coe_param_t::COE_PARAM_INT32:
							tmpval32[0] = args[3].ival;
							status = x->pcoupler->doCoEIO(1,
								x->pcontroller->m_nTerminalIndex, cparam.index, 1, tmpval32,
								cparam.subindex);
							if(status)
								epicsPrintf("Could not set %s to %u\n", cparam.name, tmpval32[0]);
							goto end;
						case coe_param_t::COE_PARAM_STRING:
						case coe_param_t::COE_PARAM_FLOAT32:
						default: goto end;
					}
				}
			}
		end:
			x->pcoupler->Unlock();
			return;
		}
	}
	epicsPrintf("Port not found.\n");
}

void el70x7PrintMessages(const iocshArgBuf* args)
{
	const char* port = args[0].sval;
	if(!port) return;
	for(auto x : controllers)
	{
		if(strcmp(x->portName, port) == 0)
		{
			x->pcoupler->Lock();
			uint16_t string[15];
			x->pcoupler->doCoEIO(0, x->pcontroller->m_nTerminalIndex, 0x1008, 5, string, 0);
			string[5] = 0;
			for(int i = 0x6; i < 0x38; i++)
			{
				x->pcoupler->doCoEIO(0, x->pcontroller->m_nTerminalIndex, 0x10f3, 14, string, i);
				string[14] = 0;
				printf("#%u: %s\n", i-0x6, (const char*)string);
			}
			x->pcoupler->Unlock();
		}
	}
}

/* Reset the motor from error state */
void el70x7ResetMotor(const iocshArgBuf* args)
{
	const char* port = args[0].sval;
	if(!port) return;
	for(auto x : controllers)
		if(strcmp(x->portName, port) == 0)
			x->getAxis(0)->output.stm_reset = 1; 
}

void el7047_Register()
{
	/* el7047Configure */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg2 = {"Port Name", iocshArgString};
		static const iocshArg arg3 = {"Record", iocshArgString};
		static const iocshArg arg4 = {"Slave position", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3, &arg4};
		static const iocshFuncDef func = {"el70x7Configure", 4, args};
		iocshRegister(&func, el7047_Configure);
	}
	/* el70x7Stat */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"el70x7Stat", 1, args};
		iocshRegister(&func, el7047_Stat);
	}
	/* el70x7SetParam */
	{
		static const iocshArg arg1 = {"EL70x7 Port Name", iocshArgString};
		static const iocshArg arg2 = {"Param Name", iocshArgString};
		static const iocshArg arg3 = {"Value", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3};
		static const iocshFuncDef func = {"el70x7SetParam", 3, args};
		iocshRegister(&func, el70x7SetParam);
	}
	/* el70x7PrintMessages */
	{
		static const iocshArg arg1 = {"Port", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"el70x7PrintMessages", 1, args};
		iocshRegister(&func, el70x7PrintMessages);
	}
	/* el70x7GetParam */
	{
		static const iocshArg arg1 = {"EL70x7 Port Name", iocshArgString};
		static const iocshArg arg2 = {"Param Name", iocshArgString};
		static const iocshArg arg3 = {"Value", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2, &arg3};
		static const iocshFuncDef func = {"el70x7GetParam", 3, args};
		iocshRegister(&func, el70x7GetParam);
	}
	/* el70x7Reset */
	{
		static const iocshArg arg0 = {"EL70x7 Port Name", iocshArgString};
		static const iocshArg* const args[] = {&arg0};
		static const iocshFuncDef func = {"el70x7Reset", 1, args};
		iocshRegister(&func, el70x7ResetMotor);
	}
}

extern "C" {
	epicsExportRegistrar(el7047_Register);
}