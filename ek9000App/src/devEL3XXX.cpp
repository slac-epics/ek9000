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
static void EL30XX_ReadCallback(CALLBACK* callback);
static long	EL30XX_dev_report(int interest);
static long EL30XX_init(int after);
static long EL30XX_init_record(void *precord);
static long EL30XX_read_record(void *precord);
static long EL30XX_linconv(void* precord, int after);

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
struct SEL30XXStandardInputPDO
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

struct SEL30XXCompactInputPDO
{
	uint16_t value;
};
#pragma pack()

struct SEL30XXSupportData
{
	CTerminal* m_pTerminal;
	CEK9000Device* m_pDevice;
	int m_nChannel;
	/* Compact or standard PDO used */
	bool m_bCompactPDO;
};

static void EL30XX_ReadCallback(CALLBACK* callback)
{
	void* record;
	SEL30XXStandardInputPDO* spdo;
	SEL30XXCompactInputPDO* cpdo;
	uint16_t buf[2];

	callbackGetUser(record, callback);
	aiRecord* pRecord = static_cast<aiRecord*>(record);
	SEL30XXSupportData* dpvt = static_cast<SEL30XXSupportData*>(pRecord->dpvt);
	free(callback);

	/* Check for invalid */
	if(!dpvt->m_pTerminal)
		return;

	/* Lock mutex */
	int status = dpvt->m_pTerminal->m_pDevice->Lock();

	if(status != epicsMutexLockOK)
	{
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		DevError("EL30XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMUTEXTIMEOUT));
		return;
	}

	/* Compact PDO sends ONLY the data, no status bits */
	if(dpvt->m_bCompactPDO)
	{
		status = dpvt->m_pTerminal->doEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->m_pTerminal->m_nInputStart +
			((dpvt->m_nChannel-1)), buf, 1);
		cpdo = reinterpret_cast<SEL30XXCompactInputPDO*>(buf);
		pRecord->rval = cpdo->value;
	}
	else
	{
		status = dpvt->m_pTerminal->doEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->m_pTerminal->m_nInputStart +
			((dpvt->m_nChannel-1) * 2), buf, 2);
		spdo = reinterpret_cast<SEL30XXStandardInputPDO*>(buf);
		pRecord->rval = spdo->value;

		/* For standard PDO types, we have limits, so we should set alarms based on these,
		   apparently the error bit is just equal to (overrange || underrange) */
		if(spdo->overrange || spdo->underrange)
		{
			recGblSetSevr(pRecord, READ_ALARM, MAJOR_ALARM);
		}
	}

	/* Set props */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	dpvt->m_pTerminal->m_pDevice->Unlock();

	/* Check for error */
	if(status)
	{
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		if(status > 0x100)
		{
			DevError("EL30XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMODBUSERR));
			return;
		}
		DevError("EL30XX_ReadCallback(): %s\n", CEK9000Device::ErrorToString(status));
		return;
	}
	return;
}

static long	EL30XX_dev_report(int interest)
{
	return 0;
}

static long EL30XX_init(int after)
{
	return 0;
}

static long EL30XX_init_record(void *precord)
{
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	pRecord->dpvt = calloc(1, sizeof(SEL30XXSupportData));
	SEL30XXSupportData* dpvt = static_cast<SEL30XXSupportData*>(pRecord->dpvt);

	/* Get the terminal */
	char* recname = NULL;
	dpvt->m_pTerminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->m_nChannel, recname);
	if(!dpvt->m_pTerminal)
	{
		Error("EL30XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}
	free(recname);

	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;
	dpvt->m_pDevice->Lock();

	/* Check connection to terminal */
	if(!dpvt->m_pTerminal->m_pDevice->VerifyConnection())
	{
		Error("EL30XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		dpvt->m_pDevice->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->m_pTerminal->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);
	dpvt->m_pDevice->Unlock();

	/* This is important; if the terminal id is different than what we want, report an error */
	if(termid != dpvt->m_pTerminal->m_nTerminalID || termid == 0)
	{
		Error("EL30XX_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}
	
	return 0;
}

static long EL30XX_read_record(void *precord)
{
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	/* Allocate and set callback */
	CALLBACK *callback = (CALLBACK *)malloc(sizeof(CALLBACK));
	*callback = *(CALLBACK*)EL30XX_ReadCallback;
	callbackSetUser(pRecord, callback);
	callbackSetCallback(EL30XX_ReadCallback, callback);
	callbackSetPriority(priorityHigh, callback);
	callbackRequest(callback);

	return 0;
}

static long EL30XX_linconv(void* precord, int after)
{
	if (1) return 0;
	aiRecord* pRecord = static_cast<aiRecord*>(precord);
	/* Max range is +/-32767 because presentation is a 16-bit signed integer */
	pRecord->eslo = (pRecord->eguf - pRecord->egul) / 0x7FFF;
	pRecord->roff = 0x0;
	return 0;
}
