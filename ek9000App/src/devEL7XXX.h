/*
 * This file is part of the EK9000 device support module. It is subject to 
 * the license terms in the LICENSE.txt file found in the top-level directory 
 * of this distribution and at: 
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
 * No part of the EK9000 device support module, including this file, may be 
 * copied, modified, propagated, or distributed except according to the terms 
 * contained in the LICENSE.txt file.
*/
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
struct SPositionInterface_Output
{
	/* 0x1601 */
	uint32_t enc_enable_lat_c : 1;
	uint32_t enc_enable_lat_epe : 1;
	uint32_t enc_set_counter : 1;
	uint32_t enc_enable_lat_ene : 1; 
	uint32_t _r1 : 12;
	uint32_t enc_set_counter_val;

	/* 0x1602 */
	uint32_t stm_enable : 1;
	uint32_t stm_reset : 1;
	uint32_t stm_reduce_torque : 1;
	uint32_t _r2 : 8;
	uint32_t stm_digout1 : 1;
	uint32_t _r3 : 4;


	/* 0x1606 */
	uint32_t pos_execute : 1;
	uint32_t pos_emergency_stp : 1;
	uint32_t _r4 : 14;
	uint32_t pos_tgt_pos;
	uint32_t pos_velocity : 16;
	uint32_t pos_start_type : 16;
	uint32_t pos_accel : 16;
	uint32_t pos_decel : 16;
};

struct SPositionInterface_Input
{
	/* 0x1A01, 0x6000 */
	uint32_t latc_valid : 1; /* Latch c valid */
	uint32_t latc_ext_valid : 1; /* Latch C extern valid */
	uint32_t cntr_set_done : 1; /* The counter was set */
	uint32_t cntr_underflow : 1;
	uint32_t cntr_overflow : 1;
	uint32_t _r1 : 2;
	uint32_t extrap_stall : 1; /* Extrapolated part of the counter was invalid */
	uint32_t stat_inp_a : 1; /* status of input a */
	uint32_t stat_inp_b : 1; /* status of inp b */
	uint32_t stat_inp_c : 1; /* Status of inp c */
	uint32_t _r2 : 1; /* 2 bits or one??? */
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
	/* 0x1A07 */
	uint32_t pos_busy : 1;
	uint32_t pos_in_tgt : 1;
	uint32_t pos_warn : 1;
	uint32_t pos_err : 1;
	uint32_t pos_calibrated : 1;
	uint32_t pos_accelerate : 1;
	uint32_t pos_decelerate : 1;
	uint32_t _r8 : 9;
	uint32_t pos_actual_pos;
	uint32_t pos_actual_vel : 16;
	uint32_t pos_actual_drive_time;
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

	devEK9000* pcoupler;
	devEK9000Terminal* pcontroller;
	drvModbusAsyn* pdrv;
	SPositionInterface_Input input;
	SPositionInterface_Output output;
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
	asynStatus move(double pos, int rel, double min_vel, double max_vel, double accel) override;

	/* Move with a velocity */
	asynStatus moveVelocity(double min_vel, double max_vel, double accel) override;
	
	/* Move to home */
	asynStatus home(double min_vel, double max_vel, double accel, int forwards) override;

	/* Stop with accel */
	asynStatus stop(double accel) override;
	
	/* Poll */
	asynStatus poll(bool* moving) override;

	/* Set the position target */
	asynStatus setPosition(double pos) override;

	/* Set the position of the encoder in steps from 0 */
	asynStatus setEncoderPosition(double pos) override;

	/* Set if it's closed loop or not */
	asynStatus setClosedLoop(bool closed) override;

	/* Report all detected motor axes */
	void report(FILE* fd, int lvl) override;

	/* Locks the driver */
	void lock();

	/* Unlocks the driver */
	void unlock();

public:
	asynStatus UpdatePDO(bool locked = false);
	asynStatus Execute(bool locked = false); /* Execute a move */
	void ResetExec();
	void ResetIfRequired();
	class el70x7Controller* pC_;
	friend class el70x7Controller;
};


/*
========================================================

class EL70X7Axis

========================================================
*/

class epicsShareClass el70x7Controller : public asynMotorController
{
public:
	devEK9000* pcoupler;
	devEK9000Terminal* pcontroller;
	el70x7Axis** paxis;
public:
	el70x7Controller(devEK9000* dev, devEK9000Terminal* controller, const char* port, int numAxis);
	
	el70x7Axis* getAxis(int num) override;
	el70x7Axis* getAxis(asynUser* axis) override;

	/* Report all parameters */
	void report(FILE* fd, int lvl) override;

	friend class el70x7Axis;
};

#define EL7047_VELO_MIN_INDEX 13
#define EL7047_VELO_MAX_INDEX 14
#define EL7047_ACCEL_POS_INDEX 15
#define EL7047_ACCEL_NEG_INDEX 16
#define EL7047_DEACCEL_POS_INDEX 17
#define EL7047_DEACCEL_NEG_INDEX 18
#define EL7047_EMERGENCY_DEACCEL_INDEX 19

#endif //_DEVEL7XXX_H_
