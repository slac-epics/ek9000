//================================================================
// Driver for the EL70XX motor terminals
// using the asyn API
//
// EL7047 Docuentation: https://download.beckhoff.com/download/document/io/ethercat-terminals/el70x7en.pdf
//================================================================

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

/* For Positioning interface compact */
/* RxPDOs : 0x1601 0x1602 0x1605 (things written to terminal by epics) */
/* TxPDOs: 0x1A01 0x1A03 0x1A06 (things read from terminal by epics) */
#pragma pack(1)
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
	uint8_t _r2 : 8;
	uint8_t stm_digout1 : 1;		/* Digital output 1 */
	uint8_t _r3 : 4;

	/* 0x1605:0x7020 */
	uint8_t pos_execute : 1;		/* Begin move with settings */
	uint8_t pos_emergency_stp : 1;	/* Emergency stop */
	uint16_t _r1 : 14;
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
#pragma pack()

/*
========================================================

class EL70X7Axis

========================================================
*/
class epicsShareClass el70x7Axis : public asynMotorAxis
{
public:
	class CouplerLock
	{
		el70x7Axis* axis;
	public:
		CouplerLock(el70x7Axis* axis);

		~CouplerLock();
	};

	CEK9000Device* pcoupler;
	CTerminal* pcontroller;
	drvModbusAsyn* pdrv;
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

	/* Report all detected motor axes */
	void report(FILE* fd, int lvl);

	/* Locks the driver */
	void lock();

	/* Unlocks the driver */
	void unlock();

private:
	asynStatus UpdatePDO(bool locked = false);
	asynStatus Execute(bool locked = false); /* Execute a move */
	asynStatus UpdateParams();
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

	/* Report all parameters */
	void report(FILE* fd, int lvl);

	friend class el70x7Axis;
};



#endif //_DEVEL7XXX_H_
