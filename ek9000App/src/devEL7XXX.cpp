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
#include <motor_interface.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "devEK9000.h"

/* For Velocity control */
/* RxPDOs: 0x1601 0x1602 0x1604
 * TxPDOs: 0x1A01 0x1A03 */
/* Mapped to the holding regs, since this is an OUTPUT */
struct SVelocityControl_Output
{
	/* From Pdo mapping ENC Control (Index 0x1601) */
	uint8_t enc_enable_lat_c;
	uint8_t enc_enable_lat_epe; /* extern on positive edge */
	uint8_t enc_set_counter;
	uint8_t enc_enable_lat_ene; /* extern on negative edge */
	uint32_t enc_counter_val;
	/* From Pdo mapping STM Control (Index 0x1602) */
	uint8_t stm_enable;
	uint8_t stm_reset;
	uint8_t stm_reduce_torque;
	uint8_t stm_digout1;
	/* From pdo mapping STM Velocity */
	uint16_t stm_velocity;
};
/* Inputs from the terminal for Velocity Control */
struct SVelocityControl_Input
{
	/* From Pdo mapping ENC Status (Index 0x1A01) */
	uint32_t enc_lat_c_valid : 1;
	uint32_t enc_ex_lat_valid : 1;
	uint32_t enc_counter_done : 1;
	uint32_t enc_counter_uf : 1; /* Underflow */
	uint32_t enc_counter_of : 1; /* overflow */
	uint32_t _r1 : 2; /* Just to align data to byte boundaries */
	uint32_t enc_extrap_stall : 1; /* Extrapolation stall */
	uint32_t enc_stat_a : 1; /* status of input a */
	uint32_t enc_stat_b : 1;
	uint32_t enc_stat_c : 1;
	uint32_t _r2 : 2; /* align */
	uint32_t enc_ext_lat_stat : 1; /* external latch status */ 
	uint32_t enc_sync_err : 1;
	uint32_t _r3 : 1;
	uint32_t enc_txpdo_toggle : 1;
	uint32_t enc_counter_val;
	uint32_t enc_latch_val;
	/* From the Pdo mapping STM Status (Index 0x1A03) */
	uint32_t stm_rdy_enable : 1; /* Ready to enable */
	uint32_t stm_rdy : 1;
	uint32_t stm_warn : 1;
	uint32_t stm_err : 1;
	uint32_t stm_mov_pos : 1;
	uint32_t stm_mov_neg : 1;
	uint32_t stm_tor_reduce : 1; /* Reduced torque */
	uint32_t stm_stall : 1; /* Motor stall */
	uint32_t _r4 : 3; /* Align */
	uint32_t stm_sync_err : 1;
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1;
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
};

/* For Velocity control compact */
/* RxPDOs: 0x1600 0x1602 0x1604
 * TxPDOs: 0x1A00 0x1A03 */
struct SVelocityControlCompact_Output
{
	/* From PDO at 0x1600 */
	uint32_t enc_enable_lat_c : 1;
	uint32_t enc_enable_lat_epe : 1; /* extern latch on positive edge */
	uint32_t enc_set_counter : 1;
	uint32_t enc_enable_lat_ene : 1; /* extern on negative edge */
	uint32_t _r1 : 12; /* Align */
	uint16_t enc_counter_val;
	/* From Pdo at 0x1602 */
	uint32_t stm_enable : 1;
	uint32_t stm_reset : 1;
	uint32_t stm_reduce_torque : 1;
	uint32_t _r2 : 8; /* Align */
	uint32_t stm_digout1 : 1;
	uint32_t _r3 : 4; /* Align */
	/* From PDO 0x1604 */
	uint16_t stm_velocity;
};

struct SVelocityControlCompact_Input
{
	/* From PDO at 0x1A00 */
	uint32_t enc_lat_c_valid : 1;
	uint32_t enc_ex_lat_valid : 1;
	uint32_t enc_counter_done : 1;
	uint32_t enc_counter_uf : 1; /* Underflow */
	uint32_t enc_counter_of : 1; /* overflow */
	uint32_t _r1 : 2; /* Just to align data to byte boundaries */
	uint32_t enc_extrap_stall : 1; /* Extrapolation stall */
	uint32_t enc_stat_a : 1; /* status of input a */
	uint32_t enc_stat_b : 1;
	uint32_t enc_stat_c : 1;
	uint32_t _r2 : 1; /* align */
	uint32_t enc_sync_err : 1; /* external latch status */ 
	uint32_t enc_stat_ex_lat : 1;
	uint32_t _r3 : 1;
	uint32_t enc_txpdo_toggle : 1;
	uint16_t enc_counter_val;
	uint16_t enc_latch_val;
	/* From the Pdo mapping STM Status (Index 0x1A03) */
	uint32_t stm_rdy_enable : 1; /* Ready to enable */
	uint32_t stm_rdy : 1;
	uint32_t stm_warn : 1;
	uint32_t stm_err : 1;
	uint32_t stm_mov_pos : 1;
	uint32_t stm_mov_neg : 1;
	uint32_t stm_tor_reduce : 1; /* Reduced torque */
	uint32_t stm_stall : 1; /* Motor stall */
	uint32_t _r4 : 3; /* Align */
	uint32_t stm_sync_err : 1;
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1;
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
};


/* For Velocity control w/ info data */
/* RxPDOs: 0x1600 0x1602 0x1604
 * TxPDOs: 0x1A00 0x1A03 0x1A04 */
struct SVelocityControlCompactEx_Output
{
	/* Pdo at 0x1600 */
	uint32_t enc_enable_lat_c : 1;
	uint32_t enc_enable_lat_epe : 1; /* extern latch on positive edge */
	uint32_t enc_set_counter : 1;
	uint32_t enc_enable_lat_ene : 1; /* extern on negative edge */
	uint32_t _r1 : 12; /* Align */
	uint16_t enc_counter_val;
	/* Pdo at 0x1602 */
	uint32_t stm_enable : 1;
	uint32_t stm_reset : 1;
	uint32_t stm_reduce_torque : 1;
	uint32_t _r2 : 8; /* Align */
	uint32_t stm_digout1 : 1;
	uint32_t _r3 : 4; /* Align */
	/* Pdo at 0x1604 */
	uint16_t stm_velocity;
};

struct SVelocityControlCompactEx_Input
{
	/* Pdo at 0x1A00 */
	uint32_t enc_lat_c_valid : 1;
	uint32_t enc_ex_lat_valid : 1;
	uint32_t enc_counter_done : 1;
	uint32_t enc_counter_uf : 1; /* Underflow */
	uint32_t enc_counter_of : 1; /* overflow */
	uint32_t _r1 : 2; /* Just to align data to byte boundaries */
	uint32_t enc_extrap_stall : 1; /* Extrapolation stall */
	uint32_t enc_stat_a : 1; /* status of input a */
	uint32_t enc_stat_b : 1;
	uint32_t enc_stat_c : 1;
	uint32_t _r2 : 1; /* align */
	uint32_t enc_sync_err : 1; /* external latch status */ 
	uint32_t enc_stat_ex_lat : 1;
	uint32_t _r3 : 1;
	uint32_t enc_txpdo_toggle : 1;
	uint16_t enc_counter_val;
	uint16_t enc_latch_val;
	/* Pdo at 0x1A03 */
	uint32_t stm_rdy_enable : 1; /* Ready to enable */
	uint32_t stm_rdy : 1;
	uint32_t stm_warn : 1;
	uint32_t stm_err : 1;
	uint32_t stm_mov_pos : 1;
	uint32_t stm_mov_neg : 1;
	uint32_t stm_tor_reduce : 1; /* Reduced torque */
	uint32_t stm_stall : 1; /* Motor stall */
	uint32_t _r4 : 3; /* Align */
	uint32_t stm_sync_err : 1;
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1;
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
	/* Pdo at 0x1A04 */
	uint16_t stm_info_dat1;
	uint16_t stm_info_dat2;
};

/* For Positioning interface compact */
/* RxPDOs: 0x1601 0x1602 0x1605 */
/* TxPDOs: 0x1A01 0x1A03 0x1A06 */
struct SPositionInterfaceCompact_Output
{
	/* 0x1601 */
	uint8_t enc_enable_lat_c;
	uint8_t enc_enable_lat_epe; /* extern on positive edge */
	uint8_t enc_set_counter;
	uint8_t enc_enable_lat_ene; /* extern on negative edge */
	uint32_t enc_counter_val;
	/* 0x1602 */
	uint8_t stm_enable;
	uint8_t stm_reset;
	uint8_t stm_reduce_torque;
	uint8_t stm_digout1;
	/* 0x1605 */
	uint32_t pos_output1 : 1;
	uint32_t pos_emergency_stp : 1;
	uint32_t _r1 : 14;
	uint32_t pos_tgt_pos;
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
	uint32_t sync_err  :1;
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
	uint32_t stm_stall : 1; /* Motor stall */
	uint32_t _r4 : 3; /* Align */
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

/* For Positioning interface */
/* RxPDOs: 0x1601 0x1602 0x1606 */
/* TxPDOs: 0x1A01 0x1A03 0x1A07 */
struct SPositionInterface_Output
{
	/* 0x1601 */
	uint8_t enc_enable_lat_epe; /* extern on positive edge */
	uint8_t enc_set_counter;
	uint8_t enc_enable_lat_ene; /* extern on negative edge */
	uint32_t enc_counter_val;
	/* 0x1602 */
	uint8_t stm_enable;
	uint8_t stm_reset;
	uint8_t stm_reduce_torque;
	uint8_t stm_digout1;
	/* 0x1606 */
	uint32_t pos_exec : 1;
	uint32_t pos_emergency_stop : 1;
	uint32_t _r1 : 14;
	uint32_t pos_tgt_pos;
	uint16_t pos_vel;
	uint16_t pos_start_typ;
	uint16_t pos_accel;
	uint32_t pos_deaccel;
};

struct SPositionInterface_Input
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
	uint32_t sync_err  :1;
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
	uint32_t stm_stall : 1; /* Motor stall */
	uint32_t _r4 : 3; /* Align */
	uint32_t stm_sync_err : 1;
	uint32_t _r5 : 1;
	uint32_t stm_txpdo_toggle : 1;
	uint32_t _r6 : 1;
	uint32_t stm_txpdo_toggle2 : 1;
	/* 0x1A07 */
	uint32_t stat_busy : 1;
	uint32_t stat_in_tgt : 1;
	uint32_t stat_warn : 1;
	uint32_t stat_err : 1;
	uint32_t stat_calibrated : 1;
	uint32_t stat_accel : 1;
	uint32_t stat_deaccel : 1;
	uint32_t _r7 : 9;
	uint32_t stat_pos;
	uint16_t stat_vel;
	uint32_t stat_drv_time;
};

static long EL7047_dev_report(int after);
static long EL7047_init(int after);
static long EL7047_init_record(void* record);
static long EL7047_get_ioint_info(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt);
static long EL7047_write_record(void* record);

struct
{
	long num;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN ioint_info;
	DEVSUPFUN write_record;
} devEL70X7 = {
	5,
	(DEVSUPFUN)EL7047_dev_report,
	NULL,
	(DEVSUPFUN)EL7047_init_record,
	(DEVSUPFUN)EL7047_get_ioint_info,
	(DEVSUPFUN)EL7047_write_record,
};

epicsExportAddress(dset, devEL70X7);

typedef struct
{
	CTerminal* m_pTerminal;
	CEK9000Device* m_pDevice;
	int m_nPdoType;
} SEL70X7SupportData;

static long EL7047_dev_report(int after)
{
	return 0;
}

static long EL7047_init(int after)
{
	return 0;
}

/* Do initialization tasks here, such as finding the record and such */
static long EL7047_init_record(void* record)
{

	motorRecord* pRecord = (motorRecord*)record;
	pRecord->dpvt = calloc(1, sizeof(SEL70X7SupportData));
	SEL70X7SupportData* dpvt = (SEL70X7SupportData*)pRecord->dpvt;
	/* Find terminal */
	char* name = NULL;
	int tmp = 0;
	dpvt->m_pTerminal = CTerminal::ProcessRecordName(pRecord->name, tmp, name);
	/* Verify that its OK */
	if(!dpvt->m_pTerminal)
	{
		Error("EL70X7_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}
	free(name);
	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;
	dpvt->m_nPdoType = dpvt->m_pTerminal->m_nPdoID; /* Make sure to record pdo type */
	/* Lock mutex */
	int status = dpvt->m_pDevice->Lock();
	if(status != epicsMutexLockOK)
	{
		Error("EL70X7_init_record(): %s\n", CEK9000Device::ErrorToString(status));
		return 1;
	}
	/* Read terminal id to make sure it matches */
	uint16_t termid = 0;
	dpvt->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);
	/* Unlock while we finish up */
	dpvt->m_pDevice->Unlock();
	if(termid == 0 || termid != dpvt->m_pTerminal->m_nTerminalID)
	{
		Error("EL70X7_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS),
				pRecord->name,
				termid);
		return 1;
	}
	/* Finish up initialization of the motor record */

	return 0;
}

static long EL7047_get_ioint_info(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt)
{
	return 0;
}

static long EL7047_write_record(void* record)
{
	return 0;
}


/*

The motor record requires a motor driver table

*/
struct driver_table el70x7_access 
{

};

static long drvEL70X7_report(int after);
static long drvEl70X7_init(void);


/*

For the EPICS driver support

*/
struct drvEL70X7_drvset
{
	long number;
	long(*report)(int);
	long(*init)(void);
} drvEL70X7 = {
	2,
	drvEL70X7_report,
	drvEl70X7_init,
};

static long drvEL70X7_report(int after)
{

}

static long drvEl70X7_init(void)
{

}