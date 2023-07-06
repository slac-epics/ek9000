/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */
//======================================================//
// Name: devEL3XXX.cpp
// Purpose: Device support for EL3xxx modules (analog in)
// Authors: Jeremy L.
// Date Created: July 6, 2019
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <devSup.h>
#include <alarm.h>
#include <aiRecord.h>
#include <recGbl.h>

#include <drvModbusAsyn.h>

#include <stddef.h>
#include <stdint.h>
#include "devEK9000.h"

#include "terminal_types.g.h"

//======================================================//
//
//	Common analog input routines
//
//======================================================//

static long EL3XXX_dev_report(int interest);
static long EL3XXX_init(int after);
static long EL3XXX_init_record(void* precord);
static long EL3XXX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt);
static long EL3XXX_linconv(void* precord, int after);

static long EL3XXX_linconv(void*, int) {
	return 0;
}

static long EL3XXX_dev_report(int) {
	return 0;
}

static long EL3XXX_init(int) {
	return 0;
}

static long EL3XXX_init_record(void* precord) {
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	pRecord->dpvt = util::allocDpvt();
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);
	uint16_t termid = 0;

	if (!util::setupCommonDpvt(pRecord, *dpvt)) {
		LOG_ERROR(dpvt->pdrv, "Unable to setup dpvt for record %s\n", pRecord->name);
		return 1;
	}

	// Read and validate terminal ID
	{
		DeviceLock lock(dpvt->pdrv);

		if (!lock.valid()) {
			LOG_ERROR(dpvt->pdrv, "unable to obtain device lock\n");
			return 1;
		}

		/* Check connection to terminal */
		if (!dpvt->pterm->m_device->VerifyConnection()) {
			LOG_ERROR(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_ENOCONN));
			return 1;
		}

		/* Check that slave # is OK */
		dpvt->pdrv->ReadTerminalID(dpvt->pterm->m_terminalIndex, termid);
	}

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->pterm->m_terminalId || termid == 0) {
		LOG_ERROR(dpvt->pdrv, "%s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}

	return 0;
}

//======================================================//
//
//	EL30XX Device support
//
//======================================================//
static long EL30XX_read_record(void* precord);

struct devEL30XX_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL30XX = {
	6,
	(DEVSUPFUN)EL3XXX_dev_report,
	(DEVSUPFUN)EL3XXX_init,
	(DEVSUPFUN)EL3XXX_init_record,
	(DEVSUPFUN)EL3XXX_get_ioint_info,
	(DEVSUPFUN)EL30XX_read_record,
	(DEVSUPFUN)EL3XXX_linconv,
};

epicsExportAddress(dset, devEL30XX);

#pragma pack(1)
// This PDO type applies to EL30XX, EL31XX and EL32XX. For 31XX and 32XX some align bits are interpreted differently.
struct EL30XXStandardInputPDO_t {
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t _r1 : 2; // First bit in this align is error for EL31XX
	uint8_t _r2 : 6; // Last bit in this align is Sync error for EL31XX
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	uint16_t value;
};
#pragma pack()

// Associate our PDO struct with the terminals
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3001);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3002);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3004);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3008);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3012);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3014);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3021);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3022);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3024);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3041);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3042);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3044);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3048);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3051);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3052);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3054);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3058);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3061);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3062);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3064);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3068);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3101);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3102);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3104);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3111);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3112);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3114);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3141);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3142);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3144);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3121);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3122);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3124);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3151);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3152);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3154);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3161);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3162);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3164);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3174);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL30XXStandardInputPDO_t, EL3202);

static long EL3XXX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);
	if (!util::DpvtValid(dpvt))
		return 1;

	*iopvt = dpvt->pdrv->m_analog_io;
	return 0;
}

static long EL30XX_read_record(void* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;
	EL30XXStandardInputPDO_t* spdo;
	uint16_t buf[2];
	/*static_assert(sizeof(buf) <= sizeof(EL30XXStandardInputPDO_t),
				  "SEL30XXStandardInputPDO is greater than 4 bytes in size, contact the author about this error!");
		*/
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!dpvt->pterm)
		return 0;

	int status;

	status = dpvt->pterm->getEK9000IO(MODBUS_READ_INPUT_REGISTERS,
									  dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 2), buf, 2);
	spdo = reinterpret_cast<EL30XXStandardInputPDO_t*>(buf);
	pRecord->rval = spdo->value;

	/* For standard PDO types, we have limits, so we should set alarms based on these,
	   apparently the error bit is just equal to (overrange || underrange) */
	if (spdo->overrange || spdo->underrange) {
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Check for error */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return 0;
		}
		LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		return 0;
	}

	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	return 0;
}

//======================================================//
//
//	EL36XX Device support
//
//======================================================//
static long EL36XX_read_record(void* precord);

struct devEL36XX_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL36XX = {
	6,
	(DEVSUPFUN)EL3XXX_dev_report,
	(DEVSUPFUN)EL3XXX_init,
	(DEVSUPFUN)EL3XXX_init_record,
	(DEVSUPFUN)EL3XXX_get_ioint_info,
	(DEVSUPFUN)EL36XX_read_record,
	(DEVSUPFUN)EL3XXX_linconv,
};
epicsExportAddress(dset, devEL36XX);

#pragma pack(1)
struct EL36XXInputPDO_t {
	uint16_t status;
	uint32_t inp;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL36XXInputPDO_t, EL3681);
DEFINE_DUMMY_OUTPUT_PDO_CHECK(EL3681); // Currently no output support for EL3681 outputs. This is a TODO!

#define EL36XX_OVERRANGE_MASK 0x2
#define EL36XX_UNDERRANGE_MASK 0x1

static long EL36XX_read_record(void* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;
	uint16_t buf[3];
	EL36XXInputPDO_t* pdo = NULL;
	/*static_assert(sizeof(EL36XXInputPDO_t) <= sizeof(buf),
				  "SEL36XXInput is greater than 3 bytes in size! Contact the author regarding this error.");
		*/
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!dpvt->pterm)
		return 0;

	/* Lock mutex */
	int status;

	status = dpvt->pterm->getEK9000IO(MODBUS_READ_INPUT_REGISTERS,
									  dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 2), buf, 2);
	pdo = reinterpret_cast<EL36XXInputPDO_t*>(buf);
	pRecord->rval = pdo->inp;

	/* Check the overrange and underrange flags */
	if ((pdo->status & EL36XX_OVERRANGE_MASK) || (pdo->status & EL36XX_UNDERRANGE_MASK)) {
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;

	/* Check for error */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return 0;
		}
		LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		return 0;
	}
	return 0;
}

//======================================================//
//
//	EL331X Device support
//
//======================================================//
static long EL331X_read_record(void* precord);

struct devEL331X_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL331X = {
	6,
	(DEVSUPFUN)EL3XXX_dev_report,
	(DEVSUPFUN)EL3XXX_init,
	(DEVSUPFUN)EL3XXX_init_record,
	(DEVSUPFUN)EL3XXX_get_ioint_info,
	(DEVSUPFUN)EL331X_read_record,
	(DEVSUPFUN)EL3XXX_linconv,
};
epicsExportAddress(dset, devEL331X);

#pragma pack(1)
struct EL331XInputPDO_t {
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t error : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	uint16_t value;
};

struct EL3314_0010_InputPDO_t {
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t error : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	uint8_t padding1 : 7; // Pad it out to byte boundary
	uint16_t value;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL331XInputPDO_t, EL3314);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL331XInputPDO_t, EL3312);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL331XInputPDO_t, EL3311);

static long EL331X_read_record(void* prec) {
	struct aiRecord* pRecord = (struct aiRecord*)prec;

	uint16_t buf[2];
	EL331XInputPDO_t* pdo = NULL;
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);

	/* Check for invalid */
	if (!dpvt->pterm)
		return 0;

	int status;

	int loc = dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 2);
	status = dpvt->pterm->getEK9000IO(MODBUS_READ_INPUT_REGISTERS, loc, buf, 2);

	pdo = reinterpret_cast<EL331XInputPDO_t*>(buf);
	pRecord->rval = pdo->value;

	/* Check the overrange and underrange flags */
	if (pdo->overrange || pdo->underrange) {
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;

	/* Check for error */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100) {

			LOG_WARNING(dpvt->pdrv, "EL331X_read_record(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return 0;
		}
		LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		return 0;
	}
	recGblSetSevr(pRecord, READ_ALARM, NO_ALARM);
	return 0;
}
