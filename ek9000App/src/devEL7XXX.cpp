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
	epicsPrintf("Locked.\n");
	if (stat) epicsAssert(__FILE__, __LINE__, "pC_->lock() != asynSuccess", "");
	asynPrint(axis->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Grabbed mutex lock for coupler=%s port=%s axisno=%u\n",
		__FUNCTION__, __LINE__, axis->pcoupler->m_pName,
		axis->pcoupler->m_pPortName, axis->axisNo_);
}

el70x7Axis::CouplerLock::~CouplerLock()
{
	this->axis->pC_->pcoupler->Unlock();
	epicsPrintf("Unlocked.\n");
	//if (stat) epicsAssert(__FILE__, __LINE__, "pC_->unlock() != asynSuccess", "");
	asynPrint(axis->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Released mutex for coupler=%s port=%s axisno=%u\n",
		__FUNCTION__, __LINE__, axis->pcoupler->m_pName,
		axis->pcoupler->m_pPortName, axis->axisNo_);
	asm("int3\n\t");
}

/*======================================================

class EL70X7Controller

NOTES:
	- None

========================================================*/

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

/*======================================================

class EL70X7Axis

NOTES:
	- The EL7047 takes acceleration in ms which represents
	the time to top speed
	- Speed is set when the class is created because it shouldn't
	change during normal operation. I do not want to be doing
	a ton of CoE I/O when we're doing semi-time critical ops
	- 

========================================================*/


el70x7Axis::el70x7Axis(el70x7Controller* pC, int axisnum) :
	asynMotorAxis(pC, axisnum)
{
	printf("el70x7Axis\n");
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
	if(this->pdrv->lock()) 
		asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to grab mutex.\n",
				__FUNCTION__, __LINE__);
	else
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Grabbed mutex.\n", 
				__FUNCTION__, __LINE__);
}

void el70x7Axis::unlock()
{
	if(this->pdrv->unlock())
		asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Unable to unlock mutex.\n", 
				__FUNCTION__, __LINE__);
	else
		asynPrint(this->pasynUser_, ASYN_TRACE_FLOW, "%s:%u Unlocked mutex.\n",
				__FUNCTION__, __LINE__);
}

asynStatus el70x7Axis::setMotorParameters(uint16_t min_start_vel,
		uint16_t max_coil_current, uint16_t reduced_coil_currrent, uint16_t nominal_voltage,
		uint16_t internal_resistance, uint16_t full_steps, uint16_t enc_inc)
{
	printf("MOTOR UPDATE PARAM\n");
	BREAK();
	this->lock();
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
	printf("MOVE\n");
	BREAK();
	this->lock();
	/* Set the params */
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	this->curr_param.forward_accel = accel;
	int stat = UpdateParams();
	if(stat) goto error;
	if(rel)
	{
		uint32_t tmp = (uint32_t)(input.cntr_val + pos);
		output.pos_tgt_pos_low = tmp & 0xFFFF;
		output.pos_tgt_pos_high = (tmp & 0xFFFF0000) >> 15;
	}
	else
	{
		//output.pos_tgt_pos = (uint32_t)pos;
		output.pos_tgt_pos_low = (uint32_t)pos & 0xFFFF;
		output.pos_tgt_pos_high = ((uint32_t)pos & 0xFFFF0000) >> 15;
	}
	output.pos_execute = 1; /* set to 1 to set counter */
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
	printf("MOVE VEL\n");
	BREAK();
	this->lock();
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
	printf("HOME\n");
	BREAK();
	this->lock();
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	if(forwards)
		this->curr_param.forward_accel = accel;
	else
		this->curr_param.back_accel = accel;
	int stat = this->UpdateParams();
	if(stat) goto error;
	/* Home is just going to be 0 for now */
	output.pos_tgt_pos[0] = 0;
	output.pos_tgt_pos[1] = 0;
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
	printf("STOP\n");
	BREAK();
	this->lock();
	/* TODO: Investigate if we have the time required to write a new accel value to the terminal */
	this->curr_param.back_accel = accel;
	int stat = this->UpdateParams();
	if(stat) goto error;
	/* TODO: We should not be using emergency stop here. */
	output.pos_emergency_stp = 1;
	stat = Execute();
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
		epicsPrintf("BRO!!!!\n");
		return asynSuccess;
	}
	this->lock();
	/* This will read params from the motor controller */
	int stat = this->UpdatePDO();
	//int stat = 0;
	if(stat) goto error;
	/* encoderposition is double */
	this->setDoubleParam(pC_->motorEncoderPosition_, (double)input.cntr_val);
	this->setIntegerParam(pC_->motorStatusDone_, input.stat_in_tgt);
	this->setIntegerParam(pC_->motorStatusDirection_, input.stm_mov_pos);
	this->setIntegerParam(pC_->motorStatusSlip_, input.stm_stall);
	this->setIntegerParam(pC_->motorStatusProblem_, input.stm_err);
	/* Check for counter overflow or underflow */
	if(input.cntr_overflow || input.cntr_underflow)
		epicsPrintf("el7047Axis::poll(): Counter overflow/underflow condition detected.\n");
	/* Checo for other error condition */
	if(input.stm_err)
		epicsPrintf("el7047Axis::poll(): Stepper motor error detected!\n");
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
	BREAK();
	this->lock();
	printf("setPosition\n");
	uint32_t rpos = (uint32_t)pos;
	output.pos_tgt_pos[0] = ((uint16_t*)&rpos)[0];
	output.pos_tgt_pos[1] = ((uint16_t*)&rpos)[1];
	output.pos_execute = 1;
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
	BREAK();
	this->lock();
	printf("setEncoderPosition\n");
	uint32_t rpos = (uint32_t)pos;
	output.enc_counter_val[0] = ((uint16_t*)&rpos)[0];
	output.enc_counter_val[1] = ((uint16_t*)&rpos)[1];
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

asynStatus el70x7Axis::UpdatePDO(bool locked)
{
	const char* pStep = "UPDATE_PDO";
	SPositionInterfaceCompact_Input old_input = this->input;
	SPositionInterfaceCompact_Output old_output = this->output;
	this->output.stm_digout1 = 1;	
	/* Update input pdos */
	int stat = pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, pcontroller->m_nInputStart,
		(uint16_t*)&this->input, pcontroller->m_nInputSize/2);
	if(stat) goto error;
	pStep = "PROPAGATE_PDO";
	/* Propagate changes from our internal pdo */
	stat = pcoupler->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_nOutputStart,
		(uint16_t*)&this->output, pcontroller->m_nOutputSize/2);
	if(stat) goto error;
	return asynSuccess;
error:
	asynPrint(this->pasynUser_, ASYN_TRACE_ERROR, "%s:%u Error while updating PDOs. Error=%u Step=%s Input Start=%u"
			" Input size=%u Output Start=%u Output Size=%u\n",
		__FUNCTION__, __LINE__, stat, pStep, pcontroller->m_nInputStart, pcontroller->m_nInputSize/2,
		pcontroller->m_nOutputStart, pcontroller->m_nOutputSize/2);
	/* Reset to previous on error */
	this->input = old_input;
	this->output = old_output;
	return asynError;
}

/*
Executes a move by setting the execute bit and propagating changes
*/
asynStatus el70x7Axis::Execute(bool locked)
{
	this->output.pos_execute = 1;
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
	printf("Param update\n");
	int res = 0;
	/* Check which params have changed and then propagate changes */
	const char* step = "";
	if(curr_param.forward_accel != prev_param.forward_accel)
	{
		step = "Update_Forward_Accel";
		printf("%s\n", step);
		/* the 7047 takes accel in the units of ms, but the motor record gives it to us in steps/s^2 */
		uint32_t data = (uint32_t)((curr_param.max_vel-curr_param.min_vel)/curr_param.forward_accel);
		res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 3);
		if(res) goto error;
		prev_param.forward_accel = curr_param.forward_accel;
	}
	if(curr_param.back_accel != prev_param.back_accel)
	{
		step = "Update_Back_Accel";
		printf("%s\n", step);
		/* same as forward accel */
		uint32_t data = (uint32_t)((curr_param.max_vel-curr_param.min_vel)/curr_param.back_accel);
		res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 4);
		if(res) goto error;
		prev_param.back_accel = curr_param.back_accel;
	}
	if(curr_param.max_vel != prev_param.max_vel)
	{
		step = "Update_Max_Vel";
		printf("%s\n", step);
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
		printf("%s\n", step);
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
	printf("Created port %s\n", port);
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

void el70x7SetParam(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	const char* port = args[1].sval;
	const char* param = args[2].sval;
	const char* val = args[3].sval;
	if(!ek9k || !port || !param || !val)
	{
		printf("maximal-current reduced-current nominal-voltage coil-resistance motor-emf motor-fullsteps motor-inductance");
		return;
	}
	for(el70x7Controller* x : controllers)
	{
		if(strcmp(port, x->portName) == 0)
		{
			x->pcoupler->Lock();

			if(strcmp(param, "maximal-current") == 0)
			{
				uint16_t v = atoi(param);
				int status = x->pcoupler->doCoEIO(1, x->pcontroller->m_nTerminalIndex, 0x8010, 0x1, &v, 0x1);
				if(status)
					printf("Could not set %s. Error=%u\n", param, status);
				else 
					printf("Set %s to %s", param, val);
			}
			else if(strcmp(param, "reduced-current") == 0)
			{

			}
			else if(strcmp(param, "nominal-voltage") == 0)
			{

			}
			else if(strcmp(param, "coil-resistance") == 0)
			{

			}
			else if(strcmp(param, "motor-emf") == 0)
			{

			}
			else if(strcmp(param, "motor-fullsteps") == 0)
			{

			}
			else if(strcmp(param, "motor-inductance") == 0)
			{

			}
			x->pcoupler->Unlock();
			return;
		}
	}
	printf("not found.\n");

}

void el70x7PrintMessages(const iocshArgBuf* args)
{
	const char* port = args[0].sval;
	for(auto x : controllers)
	{
		if(strcmp(x->portName, port) == 0)
		{
			x->pcoupler->Lock();
			uint16_t string[15];
			x->pcoupler->doCoEIO(0, x->pcontroller->m_nTerminalIndex, 0x1008, 5, string, 0);
			string[5] = 0;
			printf("%s\n", string);
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
		static const iocshArg arg0 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg1 = {"EL70x7 Port Name", iocshArgString};
		static const iocshArg arg2 = {"Param Name", iocshArgString};
		static const iocshArg arg3 = {"Value", iocshArgString};
		static const iocshArg* const args[] = {&arg0, &arg1, &arg2, &arg3};
		static const iocshFuncDef func = {"el70x7SetParam", 4, args};
		iocshRegister(&func, el70x7SetParam);
	}
	/* el70x7PrintMessages */
	{
		static const iocshArg arg1 = {"Port", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"el70x7PrintMessages", 1, args};
		iocshRegister(&func, el70x7PrintMessages);
	}
}

extern "C" {
	epicsExportRegistrar(el7047_Register);
}
