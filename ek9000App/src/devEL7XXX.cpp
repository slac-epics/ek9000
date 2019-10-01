//======================================================//
// Name: devEL7XXX.cpp
// Purpose: Device support for EL7xxx modules (motor control)
//          requires the motor record module for epics
// Authors: Jeremy L.
// Date Created: July 17, 2019
//======================================================//

/*
 * Some notes on this thingy.
 *
 * A total of 7 PDO types, each of which can be used with the module:
 * 	- velocity control 
 *	- velocity control compact
 *	- velocity control compact with extra info data
 *	- position control
 *	- positioning interface 
 *	- positioning interface compact
 *	- positioning interface compact with extra info data
 *
 * I've defined structures for each of these types below. 
 * Below is a list of fields that are used for each Pdo type:
 *
 *
 */

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
	/* 0x1A01 */
	uint32_t latc_valid : 1;
	uint32_t latc_ext_valid : 1;
	uint32_t cntr_done : 1;
	uint32_t cntr_underflow : 1;
	uint32_t cntr_overflow : 1;
	uint32_t _r1 : 2;
	uint32_t extrap_stall : 1;
	uint32_t stat_inp_a : 1;
	uint32_t stat_inp_b : 1;
	uint32_t stat_inp_c : 1;
	uint32_t _r2 : 2;
	uint32_t stat_ext_lat : 1;
	uint32_t sync_err : 1;
	uint32_t _r3 : 1;
	uint32_t txpdo_toggle : 1;
	uint32_t cntr_val;
	uint32_t lat_val;
	/* 0x1A03 */
	uint32_t stm_rdy_enable : 1; /* Ready to enable */
	uint32_t stm_rdy : 1;
	uint32_t stm_warn : 1;
	uint32_t stm_err : 1;
	uint32_t stm_mov_pos : 1;
	uint32_t stm_mov_neg : 1;
	uint32_t stm_tor_reduce : 1; /* Reduced torque */
	uint32_t stm_stall : 1;		 /* Motor stall */
	uint32_t _r4 : 3;			 /* Align */
	uint32_t stm_sync_err : 1;
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1;
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
	/* 0x1A06 */
	uint32_t stat_busy : 1;
	uint32_t stat_in_tgt : 1;
	uint32_t stat_warn : 1;
	uint32_t stat_err : 1;
	uint32_t stat_calibrated : 1;
	uint32_t stat_accel : 1;
	uint32_t stat_deaccel : 1;
	uint32_t _r7 : 9;
};
/*
Devices should be able to be accessed with O(1) time (this means using an array)
Requiring a loop thru of all devices is a poor solution.
*/

/* Copied from another deivce support... */
static msg_types EL70X7_table[] = {
	MOTION,	/* MOVE_ABS */
	MOTION,	/* MOVE_REL */
	MOTION,	/* HOME_FOR */
	MOTION,	/* HOME_REV */
	IMMEDIATE, /* LOAD_POS */
	IMMEDIATE, /* SET_VEL_BASE */
	IMMEDIATE, /* SET_VELOCITY */
	IMMEDIATE, /* SET_ACCEL */
	IMMEDIATE, /* GO */
	IMMEDIATE, /* SET_ENC_RATIO */
	INFO,	  /* GET_INFO */
	MOVE_TERM, /* STOP_AXIS */
	VELOCITY,  /* JOG */
	IMMEDIATE, /* SET_PGAIN */
	IMMEDIATE, /* SET_IGAIN */
	IMMEDIATE, /* SET_DGAIN */
	IMMEDIATE, /* ENABLE_TORQUE */
	IMMEDIATE, /* DISABL_TORQUE */
	IMMEDIATE, /* PRIMITIVE */
	IMMEDIATE, /* SET_HIGH_LIMIT */
	IMMEDIATE, /* SET_LOW_LIMIT */
	VELOCITY   /* JOB_VELOCITY */
};

struct CoECmd
{
	uint16_t index;
	uint16_t subindex;
	uint16_t val;
};

struct SEL70X7MsgData
{
	SPositionInterfaceCompact_Output pout;
	CoECmd cmds[32]; /* Modifications to values in CoE */
	CEK9000Device* dev;
	CTerminal* controller;
};


/*

For the EPICS driver support

*/
static long EL70X7_init(int after);
static long EL70X7_init_record(void *precord);
static long EL70X7_start_trans(struct motorRecord *precord);
static RTN_STATUS EL70X7_build_trans(motor_cmnd cmd, double *param, struct motorRecord *precord);
static RTN_STATUS EL70X7_end_trans(struct motorRecord *precord);

struct motor_dset devEL70X7 =
	{
		{8,
		 NULL,
		 (DEVSUPFUN)EL70X7_init,
		 (DEVSUPFUN)EL70X7_init_record,
		 NULL},
		motor_update_values,
		EL70X7_start_trans,
		EL70X7_build_trans,
		EL70X7_end_trans};
epicsExportAddress(motor_dset, devEL70X7);

/*===================================================================

=====================================================================*/
static long EL70X7_init(int after)
{
}
/*===================================================================

=====================================================================*/
static long EL70X7_init_record(void *precord)
{
}
/*===================================================================
begins a transaction, which can then be added on to.
This will grab current values from the EK9000 itself, and store them
such that they can be modified and re-written to the device.
=====================================================================*/
static long EL70X7_start_trans(struct motorRecord *precord)
{
	motor_trans* trans = (motor_trans*)precord->dpvt;
	mess_node* call = &trans->motor_call;
	char* msg = call->message;
	/* Allocate a new structure for actual processing... */
	SEL70X7MsgData* dat = (SEL70X7MsgData*)malloc(sizeof(SEL70X7MsgData));
	
	sprintf(msg, "%llu", dat);
	return 0;
}
/*===================================================================
Builds a transaction string to pass to the underlying driver support.
This is going to be as simple as possible since modbus itself is a simple
protocol.
Message reference:
Strings are all null terminated.

A: Read holding registers
B: Read input registers
C: Read coils
D: Read discrete inputs
E: Write single holding register 
F: Write multiple holding registers
G: Write coils
H: Force single coil
"A<num> addresses..."
"B<num> addresses..."
"C<num> addresses..."
"D<num> addresses..."
"E<address> value"
"F<address> values..."
"G<address> values..."
"H<address> value"
=====================================================================*/
static RTN_STATUS EL70X7_build_trans(motor_cmnd cmd, double *param, struct motorRecord *precord)
{
	motor_trans *trans = (motor_trans *)precord->dpvt;
	mess_node *call = &trans->motor_call;
	char *msg = call->message;
	int axis = call->signal;
	int card = call->card;
	CEK9000Device *dev = g_pDeviceMgr->FindDevice(card);
	if (!dev || axis > dev->m_nTerms)
	{
		return RTN_VALUES::ERROR;
	}
	CTerminal *controller = &dev->m_pTerms[call->signal];
	/* Until I find a way to share data between support functions */
	SEL70X7MsgData* dat = NULL;
	sscanf(msg, "%llu", dat);

	/* Configure the structures and whatnot before building the message */
	SPositionInterfaceCompact_Output pout;
	if (controller->m_nTerminalID == 7047 || controller->m_nTerminalID == 7037)
	{
		/* Configure the current parameters */
		
		/* Build the transaction */
		switch (cmd)
		{
		MOTION:	/* MOVE_ABS */
		MOTION:	/* MOVE_REL */
		MOTION:	/* HOME_FOR */
		MOTION:	/* HOME_REV */
		IMMEDIATE: /* LOAD_POS */
		IMMEDIATE: /* SET_VEL_BASE */
		IMMEDIATE: /* SET_VELOCITY */
		IMMEDIATE: /* SET_ACCEL */
		IMMEDIATE: /* EXECUTE! */
		{
			/* exectute order 66.. */
		}
		IMMEDIATE: /* SET_ENC_RATIO */
		{
			
		}
		INFO:	  /* GET_INFO */
		MOVE_TERM: /* STOP_AXIS */
		VELOCITY:  /* JOG */
		IMMEDIATE: /* SET_PGAIN */
		IMMEDIATE: /* SET_IGAIN */
		IMMEDIATE: /* SET_DGAIN */
		IMMEDIATE: /* ENABLE_TORQUE */
		IMMEDIATE: /* DISABLE_TORQUE */
		IMMEDIATE: /* PRIMITIVE */
		IMMEDIATE: /* SET_HIGH_LIMIT */
		IMMEDIATE: /* SET_LOW_LIMIT */
		VELOCITY:  /* JOG_VELOCITY */
		}
	}
}

/*===================================================================
For the end transaction, we actually build the string and get it
ready to be sent to the lower-level device driver. 
=====================================================================*/
static RTN_STATUS EL70X7_end_trans(struct motorRecord *precord)
{
	motor_trans *trans = (motor_trans *)precord->dpvt;
	mess_node *call = &trans->motor_call;
	char *msg = call->message;
	int axis = call->signal;
	int card = call->card;
	CEK9000Device *dev = g_pDeviceMgr->FindDevice(card);
	if (!dev || axis > dev->m_nTerms)
	{
		return RTN_VALUES::ERROR;
	}
	CTerminal *controller = &dev->m_pTerms[call->signal];
	SPositionInterfaceCompact_Output pout;

	SEL70X7MsgData* dat = NULL;
	sscanf(msg, "%llu", &dat);

	/* Build the message from the pout struct */
	uint16_t outstart = controller->m_nOutputStart;
	sprintf(msg, "F%u", outstart); /* Write multiple holding registers */
	for(int i = 0; i < sizeof(pout)/2; i++)
		sprintf(msg, "%s %u", ((uint16_t*)&pout)[i]); /* Append to string */
}

/*

The motor record requires a motor driver table

*/
static int recv_mess(int card, char *msg, int flags);
static RTN_STATUS send_mess(int card, const char *msg, char *name);
static int set_status(int card, int signal);
static long report(int);
static long init();
static int motor_init();
static void fillCmndInfo();
static void start_status(int);
static void query_done(int, int, struct mess_node *);

struct driver_table el70x7_access
{
	motor_init,
		motor_send,
		motor_free,
		motor_card_info,
		motor_axis_info,
		&mess_queue,
		&queue_lock,
		&free_list,
		&freelist_lock,
		&motor_sem,
		&motor_state,
		&total_cards,
		&any_motor_in_motion,
		send_mess,
		recv_mess,
		set_status,
		query_done,
		NULL,
		&initialized,
		NULL
};

/*===================================================================
This function is used to send a signal to the specified card.
msg is a string that represents the message to be sent.
Because this driver uses modbus, this command will need 
to parse and process the strings it's passed.
=====================================================================*/
static RTN_STATUS send_mess(int card, char *msg, char *name)
{
}

/*===================================================================
This function is called when a message should be received from 
a motor terminal. It should process the modbus returned messages and
turn it into a string.
=====================================================================*/
static int recv_mess(int card, char *msg, int flags)
{
}

/*===================================================================
This function should set the status of an axis. Card is the ID of the
bus coupler. signal is the slave # of the motor terminal.
=====================================================================*/
static int set_status(int card, int signal)
{
}
/*===================================================================

=====================================================================*/
static long report(int)
{
}

/*===================================================================

=====================================================================*/
static void query_done(int, int, struct mess_node *)
{
}

class epicsShareClass el70x7Controller : public asynMotorController
{
public:
	CEK9000Device* pcoupler;
	CTerminal* pcontroller;
	el70x7Axis** paxis;
	uint32_t accel_forward;
	uint32_t accel_back; /* Stopping accel basically */
	uint32_t max_vel;
	uint32_t min_vel;
public:
	el70x7Controller(CEK9000Device* dev, CTerminal* controller, const char* port, int numAxis);
	
	class el70x7Axis* getAxis(int num);
	class el70x7Axis* getAxis(asynUser* axis);
};

class epicsShareClass el70x7Axis : public asynMotorAxis
{
private:
	CEK9000Device* pcoupler;
	CTerminal* pcontroller;
	SPositionInterfaceCompact_Input input;
	SPositionInterfaceCompact_Output output;
public:
	el70x7Axis(el70x7Controller* pC, int axisno);

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

	/* Set if it's closed loop or not */
	asynStatus setClosedLoop(bool closed);

private:
	void SetVelocity(double vel);
	void SetAcceleration(double accel);
	void UpdatePDO();
	void Execute(); /* Execute a move */
	/* Update CoE parameters */
	/* 0 = all */
	/* 1 = max_vel, 2 = min_vel, 3 = accel_forward, 4 = accel_back */
	void UpdateParams(int params = 0);
};

el70x7Controller::el70x7Controller(CEK9000Device* dev, CTerminal* controller, const char* port, int numAxis) :
	asynMotorController(port, numAxis, 0, 0, 0, ASYN_MULTIDEVICE, 0, 0, 0),
	pcoupler(dev),
	pcontroller(controller)
{

}

el70x7Axis* el70x7Controller::getAxis(int num)
{
	return this->paxis[num];
}

el70x7Axis* el70x7Controller::getAxis(asynUser* usr)
{
}

/*======================================================

class EL70X7Axis

========================================================*/


el70x7Axis::el70x7Axis(el70x7Controller* pC, int axisnum) :
	el70x7Axis(pC, axisnum)
{
	this->pcoupler = pC->pcoupler;
	this->pcontroller = pC->pcontroller;
	/* Read all pdo data from the device */
	this->pcoupler->Lock();
	/* Grab initial values */
	this->pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 
		pcontroller->m_nOutputStart, (uint16_t*)&this->output, pcontroller->m_nOutputSize/2);
	this->pcoupler->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS,
		pcontroller->m_nInputStart, (uint16_t*)&this->input, pcontroller->m_nInputSize/2);
	this->pcoupler->Unlock();
}

asynStatus el70x7Axis::move(double pos, int rel, double min_vel, double max_vel, double accel)
{
	this->SetAcceleration(accel);
	
}

asynStatus el70x7Axis::moveVelocity(double min_vel, double max_vel, double accel)
{
	this->SetAcceleration(accel);
}

asynStatus el70x7Axis::home(double min_vel, double max_vel, double accel, int forwards)
{
	this->SetAcceleration(accel);
}

asynStatus el70x7Axis::stop(double accel)
{
	/* TODO: Investigate if we have the time required to write a new accel value to the terminal */
	
}

asynStatus el70x7Axis::poll(bool* moving)
{
	/* Polling thread implemented by the ek9000 driver itself */
}

asynStatus el70x7Axis::setPosition(double pos)
{
	/* Validate the parameters to ensure we aren't going to screw ourselves */
	if(pos < 0)
		return asynError;
	this->output.pos_tgt_pos = (uint32_t)pos;
	return asynSuccess;
}

asynStatus el70x7Axis::setClosedLoop(bool closed) 
{

}

void el70x7Axis::UpdatePDO()
{
	this->pcoupler->Lock();
	/* Update input pdos */
	pcoupler->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_nInputStart,
		(uint16_t*)&this->input, pcontroller->m_nInputSize/2);
	/* Propagate changes from our internal pdo */
	pcoupler->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, pcontroller->m_nOutputStart,
		(uint16_t*)&this->output, pcontroller->m_nOutputSize/2);
	this->pcoupler->Unlock();
}

void el70x7Axis::Execute()
{
	this->output.pos_execute = 1;
	this->UpdatePDO();
}

void el70x7Axis::UpdateParams(int param)
{
	switch(param)
	{
		default:
			
			break;
	}
}