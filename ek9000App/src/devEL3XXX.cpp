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
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
#include <aiRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <alarm.h>
#include <recGbl.h>

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "devEK9000.h"

//======================================================//
//
//	EL30XX Device support
//
//======================================================//
static void EL30XX_ReadCallback(CALLBACK *callback);
static long EL30XX_dev_report(int interest);
static long EL30XX_init(int after);
static long EL30XX_init_record(void *precord);
static long EL30XX_read_record(void *precord);
static long EL30XX_linconv(void *precord, int after);

struct
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL30XX = {
    6,
    (DEVSUPFUN)EL30XX_dev_report,
    (DEVSUPFUN)EL30XX_init,
    (DEVSUPFUN)EL30XX_init_record,
    NULL,
    (DEVSUPFUN)EL30XX_read_record,
    (DEVSUPFUN)EL30XX_linconv,
};

epicsExportAddress(dset, devEL30XX);

#pragma pack(1)
/*
Okay beckhoff, what the h*ck! The status bits COME AFTER THE VALUE 
but the docs say they come before! What! 
Never had this issue with ANY other terminal type. I feel dirty and wrong
for swapping the bytes around here.
*/
struct EL30XXStandardInputPDO_t
{
	uint8_t underrange : 1;
	uint8_t overrange : 1;
	uint8_t limit1 : 2;
	uint8_t limit2 : 2;
	uint8_t _r1 : 2;
	uint8_t _r2 : 6;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
	uint16_t value;
};

struct EL30XXCompactInputPDO_t
{
	uint16_t value;
};
#pragma pack()

/* Passed to device private */
struct EL30XXDPVT_t
{
	CTerminal *terminal;
	CEK9000Device *device;
	int channel;
	/* Compact or standard PDO used */
	bool compactPDO;
	terminal_dpvt_t newDpvt;
};

static void EL30XX_ReadCallback(CALLBACK *callback)
{
	void *record;
	EL30XXStandardInputPDO_t *spdo;
	EL30XXCompactInputPDO_t *cpdo;
	uint16_t buf[2];
	static_assert(sizeof(buf) <= sizeof(EL30XXStandardInputPDO_t), "SEL30XXStandardInputPDO is greater than 4 bytes in size, contact the author about this error!");

	callbackGetUser(record, callback);
	aiRecord *pRecord = static_cast<aiRecord *>(record);
	EL30XXDPVT_t *dpvt = static_cast<EL30XXDPVT_t *>(pRecord->dpvt);
	free(callback);

	/* Check for invalid */
	if (!dpvt->terminal)
		return;

	/* Lock mutex */
	int status = dpvt->terminal->m_pDevice->Lock();

	if (status != epicsMutexLockOK)
	{
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		DevError("EL30XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMUTEXTIMEOUT));
		return;
	}

	/* Compact PDO sends ONLY the data, no status bits */
	if (dpvt->compactPDO)
	{
		status = dpvt->terminal->doEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->terminal->m_nInputStart + ((dpvt->channel - 1)), buf, 1);
		cpdo = reinterpret_cast<EL30XXCompactInputPDO_t *>(buf);
		pRecord->rval = cpdo->value;
	}
	else
	{
		status = dpvt->terminal->doEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->terminal->m_nInputStart + ((dpvt->channel - 1) * 2), buf, 2);
		spdo = reinterpret_cast<EL30XXStandardInputPDO_t *>(buf);
		pRecord->rval = spdo->value;

		/* For standard PDO types, we have limits, so we should set alarms based on these,
		   apparently the error bit is just equal to (overrange || underrange) */
		if (spdo->overrange || spdo->underrange)
		{
			recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
		}
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	dpvt->terminal->m_pDevice->Unlock();

	/* Check for error */
	if (status)
	{
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100)
		{
			DevError("EL30XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMODBUSERR));
			return;
		}
		DevError("EL30XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(status));
		return;
	}
	return;
}

static long EL30XX_dev_report(int interest)
{
	return 0;
}

static long EL30XX_init(int after)
{
	return 0;
}

static long EL30XX_init_record(void *precord)
{
	aiRecord *pRecord = static_cast<aiRecord *>(precord);
	pRecord->dpvt = calloc(1, sizeof(EL30XXDPVT_t));
	EL30XXDPVT_t *dpvt = static_cast<EL30XXDPVT_t *>(pRecord->dpvt);

	dpvt->newDpvt = CEK9000Device::emptyDpvt();
        CEK9000Device::setupCommonDpvt<aiRecord>(pRecord, dpvt->newDpvt);
	

	/* Get the terminal */
	char *recname = NULL;
	dpvt->terminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->channel, recname);
	if (!dpvt->terminal)
	{
		Error("EL30XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}
	free(recname);

	dpvt->device = dpvt->terminal->m_pDevice;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->terminal->m_pDevice->VerifyConnection())
	{
		Error("EL30XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_pDevice->ReadTerminalID(dpvt->terminal->m_nTerminalIndex, termid);
	dpvt->device->Unlock();

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->terminal->m_nTerminalID || termid == 0)
	{
		Error("EL30XX_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}

	return 0;
}

static long EL30XX_read_record(void *precord)
{
	util::setupCallback(precord, EL30XX_ReadCallback);
	return 0;
}

static long EL30XX_linconv(void *precord, int after)
{
	return 0;
}

//======================================================//
//
//	EL36XX Device support
//
//======================================================//
static void EL36XX_ReadCallback(CALLBACK *callback);
static long EL36XX_dev_report(int interest);
static long EL36XX_init(int after);
static long EL36XX_init_record(void *precord);
static long EL36XX_read_record(void *precord);
static long EL36XX_linconv(void *precord, int after);

struct
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
	DEVSUPFUN linconv;
} devEL36XX = {
	6,
	(DEVSUPFUN)EL36XX_dev_report,
	(DEVSUPFUN)EL36XX_init,
	(DEVSUPFUN)EL36XX_init_record,
	NULL,
	(DEVSUPFUN)EL36XX_read_record,
	(DEVSUPFUN)EL36XX_linconv,
};
epicsExportAddress(dset, devEL36XX);

struct EL36XXDpvt_t
{
	CTerminal *terminal;
	CEK9000Device *device;
	int channel;
};

#pragma pack(1)
struct EL36XXInputPDO_t
{
	uint32_t inp;
	uint16_t status;
};
#pragma pack()

#define EL36XX_OVERRANGE_MASK 0x2
#define EL36XX_UNDERRANGE_MASK 0x1

static void EL36XX_ReadCallback(CALLBACK *callback)
{
	void *record;
	uint16_t buf[3];
	EL36XXInputPDO_t* pdo = NULL;
	static_assert(sizeof(EL36XXInputPDO_t) <= sizeof(buf), "SEL36XXInput is greater than 3 bytes in size! Contact the author regarding this error.");

	callbackGetUser(record, callback);
	aiRecord *pRecord = static_cast<aiRecord *>(record);
	EL36XXDpvt_t *dpvt = static_cast<EL36XXDpvt_t *>(pRecord->dpvt);
	free(callback);

	/* Check for invalid */
	if (!dpvt->terminal)
		return;

	/* Lock mutex */
	int status = dpvt->terminal->m_pDevice->Lock();

	if (status != epicsMutexLockOK)
	{
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		DevError("EL36XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMUTEXTIMEOUT));
		return;
	}

	status = dpvt->terminal->doEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->terminal->m_nInputStart + ((dpvt->channel - 1) * 2), buf, 2);
	pdo = reinterpret_cast<EL36XXInputPDO_t*>(buf);
	pRecord->rval = pdo->inp;

	/* Check the overrange and underrange flags */
	if ((pdo->status & EL36XX_OVERRANGE_MASK) || (pdo->status & EL36XX_UNDERRANGE_MASK))
	{
		recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	dpvt->terminal->m_pDevice->Unlock();

	/* Check for error */
	if (status)
	{
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if (status > 0x100)
		{
			DevError("EL36XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMODBUSERR));
			return;
		}
		DevError("EL36XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(status));
		return;
	}
	return;
}

static long EL36XX_dev_report(int interest)
{
	return 0;
}

static long EL36XX_init(int after)
{
	return 0;
}

static long EL36XX_init_record(void *precord)
{
	aiRecord *pRecord = static_cast<aiRecord *>(precord);
	pRecord->dpvt = calloc(1, sizeof(EL36XXDpvt_t));
	EL36XXDpvt_t *dpvt = static_cast<EL36XXDpvt_t *>(pRecord->dpvt);

	/* Get the terminal */
	char *recname = NULL;
	dpvt->terminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->channel, recname);
	if (!dpvt->terminal)
	{
		Error("EL36XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}
	free(recname);

	dpvt->device = dpvt->terminal->m_pDevice;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->terminal->m_pDevice->VerifyConnection())
	{
		Error("EL36XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_pDevice->ReadTerminalID(dpvt->terminal->m_nTerminalIndex, termid);
	dpvt->device->Unlock();

	/* This is important; if the terminal id is different than what we want, report an error */
	if (termid != dpvt->terminal->m_nTerminalID || termid == 0)
	{
		Error("EL36XX_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}

	return 0;
}

static long EL36XX_read_record(void *precord)
{
	util::setupCallback(precord, EL36XX_ReadCallback);
	return 0;
}

static long EL36XX_linconv(void *precord, int after)
{
	return 0;
}
