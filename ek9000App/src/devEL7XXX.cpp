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

#include "devEK9000.h"

/* For Positioning interface compact */
/* RxPDOs: 0x1601 0x1602 0x1605 */
/* TxPDOs: 0x1A01 0x1A03 0x1A06 */
struct SPositionInterfaceCompact_Output
{
	/* 0x1601 */
	uint8_t enc_enable_lat_c : 1;	/* Enable latching thru the C track */
	uint8_t enc_enable_lat_epe : 1; /* extern on positive edge */
	uint8_t enc_set_counter : 1;	/* Set the counter value */
	uint8_t enc_enable_lat_ene : 1; /* extern on negative edge */
	uint16_t _r4 : 12;
	uint32_t enc_counter_val;	/* The value to be set thru set counter */
	/* 0x1602:0x7010 */
	uint8_t stm_enable	: 1;			/* Enable output stage */
	uint8_t stm_reset	: 1;			/* Reset and clear all errors */
	uint8_t stm_reduce_torque : 1;		/* Recuded coil current is active */
	uint8_t _r2;
	uint8_t stm_digout1 : 1;		/* Digital output 1 */
	uint8_t _r3 : 4;
	/* 0x1605:0x7020 */
	uint32_t pos_execute : 1;		/* Begin move with settings */
	uint32_t pos_emergency_stp : 1;	/* Emergency stop */
	uint32_t _r1 : 14;
	uint32_t pos_tgt_pos;			/* Target position */
};

struct SPositionInterfaceCompact_Input
{
	/* 0x1A01, 0x6000 */
	uint32_t latc_valid : 1; /* Latch c valid */
	uint32_t latc_ext_valid : 1; /* Latch C extern valid */
	uint32_t cntr_done : 1; /* The counter was set */
	uint32_t cntr_underflow : 1;
	uint32_t cntr_overflow : 1;
	uint32_t _r1 : 2;
	uint32_t extrap_stall : 1; /* Extrapolated part of the counter was invalid */
	uint32_t stat_inp_a : 1; /* status of input a */
	uint32_t stat_inp_b : 1; /* status of inp b */
	uint32_t stat_inp_c : 1; /* Status of inp c */
	uint32_t _r2 : 2;
	uint32_t stat_ext_lat : 1; /* Status of extern latch */
	uint32_t sync_err : 1;	/* Sync error */
	uint32_t _r3 : 1;
	uint32_t txpdo_toggle : 1; /* Toggled when data is updated */
	uint32_t cntr_val; /* The counter value */
	uint32_t lat_val; /* Latch value */
	/* 0x1A03, 0x6010 */
	uint32_t stm_rdy_enable : 1; /* Driver stage is ready for enabling */
	uint32_t stm_rdy : 1; /* Driver stage is ready for operation */
	uint32_t stm_warn : 1; /* warning has happened */
	uint32_t stm_err : 1; /* Error has happened */
	uint32_t stm_mov_pos : 1; /* Moving in the positive dir */
	uint32_t stm_mov_neg : 1; /* Moving in the negative dir */
	uint32_t stm_tor_reduce : 1; /* Reduced torque */
	uint32_t stm_stall : 1;		 /* Motor stall */
	uint32_t _r4 : 3;
	uint32_t stm_sync_err : 1; /* Set if synchronization error in previous step */
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1; /* txpdo toggle  */
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
	/* 0x1A06, 0x6020 */
	uint32_t stat_busy : 1; /* current travel command is active */
	uint32_t stat_in_tgt : 1; /* Motor has arrived */
	uint32_t stat_warn : 1; /* Warning happened */
	uint32_t stat_err : 1; /* Error happened */
	uint32_t stat_calibrated : 1; /* Motor is calibrated */
	uint32_t stat_accel : 1; /* Motor is accelerating */
	uint32_t stat_deaccel : 1; /* Motor is slowing down */
	uint32_t _r7 : 9;
};
/*======================================================

class EL70X7Axis

========================================================*/
class epicsShareClass el70x7Axis : public asynMotorAxis
{
private:
	CEK9000Device* pcoupler;
	CTerminal* pcontroller;
	SPositionInterfaceCompact_Input input;
	SPositionInterfaceCompact_Output output;
	struct {
		double forward_accel;
		double back_accel;
		double max_vel;
		double min_vel;
	} curr_param, prev_param;
	/* This parameter should not be changed during operation of the motor */
	/* FOR NOW AT LEAST */
	uint32_t speed;
	uint32_t enc_pos;
public:
	el70x7Axis(class el70x7Controller* pC, int axisno);

	/*
	min_start_vel: Min starting velocity of the motor (10,000 = 100%)
	max_coil_current: Max current passing thru the motor coils (mA)
	reduced_coil_current: (mA)
	nominal_volage: operating voltage of the motor (mV)
	internal_resistance: internal resistance of the motor (10mOhm)
	full_steps: Number of full motor steps
	enc_inc: the number of increments of the encoder per revolution (4-fold)
	*/
	asynStatus setMotorParameters(uint16_t min_start_vel,
		uint16_t max_coil_current, uint16_t reduced_coil_currrent, uint16_t nominal_voltage,
		uint16_t internal_resistance, uint16_t full_steps, uint16_t enc_inc);

	/* Move to home */
	asynStatus move(double pos, int rel, double min_vel, double max_vel, double accel);

	/* Move with a velocity */
	asynStatus moveVelocity(double min_vel, double max_vel, double accel);
	
	/* Move to home */
	asynStatus home(double min_vel, double max_vel, double accel, int forwards);

	/* Stop with accel */
	asynStatus stop(double accel);
	
	/* Poll */
	asynStatus poll(bool* moving);

	/* Set the position target */
	asynStatus setPosition(double pos);

	/* Set the position of the encoder in steps from 0 */
	asynStatus setEncoderPosition(double pos);

	/* Set if it's closed loop or not */
	asynStatus setClosedLoop(bool closed);

private:
	void UpdatePDO(bool locked = false);
	void Execute(bool locked = false); /* Execute a move */
	void UpdateParams();
	class el70x7Controller* pC_;
	friend class el70x7Controller;
};


/*======================================================

class EL70X7Axis

========================================================*/

class epicsShareClass el70x7Controller : public asynMotorController
{
public:
	CEK9000Device* pcoupler;
	CTerminal* pcontroller;
	el70x7Axis** paxis;
public:
	el70x7Controller(CEK9000Device* dev, CTerminal* controller, const char* port, int numAxis);
	
	el70x7Axis* getAxis(int num);
	el70x7Axis* getAxis(asynUser* axis);

	friend class el70x7Axis;
};


/*======================================================

class EL70X7Controller

NOTES:
	- None

========================================================*/

el70x7Controller::el70x7Controller(CEK9000Device* dev, CTerminal* controller, const char* port, int numAxis) :
	asynMotorController(port, numAxis, 0, 0, 0, ASYN_MULTIDEVICE | ASYN_CANBLOCK, 0, 0, 0),
	pcoupler(dev),
	pcontroller(controller)
{
}

el70x7Axis* el70x7Controller::getAxis(int num)
{
	if(num >= this->numAxes_ || num < 0)
		return NULL;
	return this->paxis[num];
}

el70x7Axis* el70x7Controller::getAxis(asynUser* usr)
{
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
	pC_= pC;
	this->pcoupler = pC->pcoupler;
	this->pcontroller = pC->pcontroller;
	/* Set previous params to random values */
	/* Grab initial values */
	this->pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 
		pcontroller->m_nOutputStart, (uint16_t*)&this->output, pcontroller->m_nOutputSize/2);
	this->pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS,
		pcontroller->m_nInputStart, (uint16_t*)&this->input, pcontroller->m_nInputSize/2);
	/* Read the configured speed */
	uint16_t spd = 0;
	this->pcoupler->doCoEIO(0, pcontroller->m_nTerminalIndex, 0x8012, 1, &spd, 0x05);
	switch(spd)
	{
		case 0:
			speed = 1000;
			break;
		case 1:
			speed = 2000;
			break;
		case 2:
			speed = 4000;
			break;
		case 3:
			speed = 8000;
			break;
		case 4:
			speed = 16000;
			break;
		case 5:
			speed = 32000;
			break;
		default:
			speed = 1000;
			break;
	}
	
}

asynStatus el70x7Axis::setMotorParameters(uint16_t min_start_vel,
		uint16_t max_coil_current, uint16_t reduced_coil_currrent, uint16_t nominal_voltage,
		uint16_t internal_resistance, uint16_t full_steps, uint16_t enc_inc)
{
	/* We assume the device is not locked */
	this->pcoupler->Lock();
	uint16_t tid = pcontroller->m_nTerminalIndex;
	int stat = 0;
	stat = pcoupler->doCoEIO(1, tid, 0x8010, 1, &max_coil_current, 0x1);
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
	return asynSuccess;
error:
	epicsPrintf("Failed to propagate some CoE parameters for motor!\n");
	return asynError;
}

asynStatus el70x7Axis::move(double pos, int rel, double min_vel, double max_vel, double accel)
{
	/* Set the params */
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	this->curr_param.forward_accel = accel;
	UpdateParams();
	if(rel)
	{
		output.pos_tgt_pos = (uint32_t)(input.cntr_val + pos);
	}
	else
		output.pos_tgt_pos = (uint32_t)pos;
	output.enc_set_counter = 1; /* set to 1 to set counter */
	/* Execute move */
	Execute();
	return asynSuccess;
}

/*
Move the motor at a constant velocity until the stop signal is recieved.
min_vel is the starting velocity of the motor, max_vel is the max velocity of the motor
the velocity is in steps/s
accel is the acceleration of the motor in steps/s^2
*/
asynStatus el70x7Axis::moveVelocity(double min_vel, double max_vel, double accel)
{
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	this->curr_param.forward_accel = accel;
	this->UpdateParams();
	/* Set velocity params */
	return asynSuccess;
}

/*
Move the motor to it's home position
*/
asynStatus el70x7Axis::home(double min_vel, double max_vel, double accel, int forwards)
{
	this->curr_param.max_vel = max_vel;
	this->curr_param.min_vel = min_vel;
	if(forwards)
		this->curr_param.forward_accel = accel;
	else
		this->curr_param.back_accel = accel;
	this->UpdateParams();
	/* Home is just going to be 0 for now */
	output.pos_tgt_pos = 0;
	Execute();
	return asynSuccess;
}

asynStatus el70x7Axis::stop(double accel)
{
	/* TODO: Investigate if we have the time required to write a new accel value to the terminal */
	this->curr_param.back_accel = accel;
	this->UpdateParams();
	/* TODO: We should not be using emergency stop here. */
	output.pos_emergency_stp = 1;
	Execute();
	return asynSuccess;
}

asynStatus el70x7Axis::poll(bool* moving)
{
	/* This will read params from the motor controller */
	this->UpdatePDO();
	this->setIntegerParam(pC_->motorEncoderPosition_, input.cntr_val);
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
	return asynSuccess;
}

asynStatus el70x7Axis::setPosition(double pos)
{
	output.enc_counter_val = (uint32_t)pos;
	output.enc_set_counter = 1;
	UpdatePDO();
	return asynSuccess;
}

asynStatus el70x7Axis::setEncoderPosition(double pos)
{
	output.enc_counter_val = (uint32_t)pos;
	output.enc_set_counter = 1;
	UpdatePDO();
	return asynSuccess;
}

asynStatus el70x7Axis::setClosedLoop(bool closed) 
{
	return asynSuccess;
}

void el70x7Axis::UpdatePDO(bool locked)
{
	/* Update input pdos */
	pcoupler->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_nInputStart,
		(uint16_t*)&this->input, pcontroller->m_nInputSize/2);
	/* Propagate changes from our internal pdo */
	pcoupler->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_nOutputStart,
		(uint16_t*)&this->output, pcontroller->m_nOutputSize/2);
}

/*
Executes a move by setting the execute bit and propagating changes
*/
void el70x7Axis::Execute(bool locked)
{
	this->output.pos_execute = 1;
	this->UpdatePDO(locked);
}

/*
Update the specified parameters
*/
void el70x7Axis::UpdateParams()
{
	/* Check which params have changed and then propagate changes */
	if(curr_param.forward_accel != prev_param.forward_accel)
	{
		/* the 7047 takes accel in the units of ms, but the motor record gives it to us in steps/s^2 */
		uint32_t data = (uint32_t)((curr_param.max_vel-curr_param.min_vel)/curr_param.forward_accel);
		//uint32_t data = (uint32_t)curr_param.forward_accel;
		int res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 3);
		prev_param.forward_accel = prev_param.forward_accel;
	}
	if(curr_param.back_accel != prev_param.back_accel)
	{
		/* same as forward accel */
		uint32_t data = (uint32_t)((curr_param.max_vel-curr_param.min_vel)/curr_param.back_accel);
		int res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 4);
		prev_param.back_accel = curr_param.back_accel;
	}
	if(curr_param.max_vel != prev_param.max_vel)
	{
		/* The speed is set on init of this axis, and it should not change during operation */
		/* The el7047 takes speed in terms of the percentage of the max velocity, multiplied by 10,000 (it likes integers) */
		uint32_t data = (uint32_t)((curr_param.max_vel/(double)this->speed)*10000);
		int res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 2);
		prev_param.max_vel = curr_param.max_vel;
	}
	if(curr_param.min_vel != prev_param.min_vel)
	{
		uint32_t data = (uint32_t)((curr_param.min_vel/(double)this->speed)*10000);
		int res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8020, 2, (uint16_t*)&data, 1);
		/* Need to propagate to STM Settings Part 1, 0x9: starting velocity */
		//res = pcoupler->doCoEIO(1, pcontroller->m_nTerminalIndex, 0x8010, 2, (uint16_t*)&data, 0x9);
		prev_param.min_vel = curr_param.min_vel;
	}
}

void el7047_Configure(const iocshArgBuf* args)
{

}

void el7047_Register()
{
	/* el7047Configure */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg2 = {"Slave position", iocshArgInt};
		static const iocshArg* args = {&arg1, &arg2};
		static const iocshFuncDef func = {"el7047Configure", 2, &args};
		iocshRegister(&func, el7047_Configure);
	}
}

extern "C" {
	epicsExportRegistrar(el7047_Register);
}