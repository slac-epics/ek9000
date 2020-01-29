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
// Name: devEL4XXX.cpp
// Purpose: Device support for EL4xxx modules (analog out)
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
#include <aoRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <recGbl.h>
#include <alarm.h> 

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "devEK9000.h"

struct SEL40XXSupportData
{
	int m_nChannel;
	CTerminal* m_pTerminal;
	CEK9000Device* m_pDevice;
	/* Standard or compact PDO used */
	bool m_bCompactPDO;
};

static long EL40XX_dev_report(int after);
static long EL40XX_init(int after);
static long EL40XX_init_record(void* record);
static long EL40XX_write_record(void* record);
static long EL40XX_linconv(void* precord, int after);

struct
{
	long num;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN ioint_info;
	DEVSUPFUN write_record;
	DEVSUPFUN linconv;
} devEL40XX = {
	6,
	(DEVSUPFUN)EL40XX_dev_report,
	(DEVSUPFUN)EL40XX_init,
	(DEVSUPFUN)EL40XX_init_record,
	NULL,
	(DEVSUPFUN)EL40XX_write_record,
	(DEVSUPFUN)EL40XX_linconv,
};

epicsExportAddress(dset, devEL40XX);

static void EL40XX_WriteCallback(CALLBACK* callback)
{
	void* record = NULL;
	callbackGetUser(record, callback);
	aoRecord* pRecord = (aoRecord*)record;
	SEL40XXSupportData* dpvt = (SEL40XXSupportData*)pRecord->dpvt;
	free(callback);

	/* Check for invalid */
	if(!dpvt->m_pTerminal)
		return;

	/* Verify connection */
	if(!dpvt->m_pTerminal->m_pDevice->VerifyConnection())
	{
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		DevError("EL40XX_WriteCallback(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		pRecord->pact = 0;
		return;
	}

	/* Lock the mutex */
	dpvt->m_pTerminal->m_pDevice->Lock();
	
	/* Set buffer & do write */
	uint16_t buf = (int16_t)pRecord->rval;
	int status = dpvt->m_pTerminal->doEK9000IO(MODBUS_WRITE_MULTIPLE_REGISTERS, dpvt->m_pTerminal->m_nOutputStart + 
		(dpvt->m_nChannel-1), &buf, 1);
	
	/* Unlock mutex */
	dpvt->m_pTerminal->m_pDevice->Unlock();

	/* No more processing */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	/* Check error */
	if(status)
	{
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if(status > 0x100)
		{
			DevError("EL40XX_WriteCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMODBUSERR));
			return;
		}
		else
		{
			DevError("EL40XX_WriteCallback(): %s\n", CEK9000Device::ErrorToString(status));
		}
		return;
	}
}

static long EL40XX_dev_report(int after)
{
	return 0;
}

static long EL40XX_init(int after)
{
	return 0;
}

static long EL40XX_init_record(void* record)
{
	aoRecord* pRecord = (aoRecord*)record;
	pRecord->dpvt = calloc(1, sizeof(SEL40XXSupportData));
	SEL40XXSupportData* dpvt = (SEL40XXSupportData*)pRecord->dpvt;

	/* Find record name */
	char* out = NULL;
	dpvt->m_pTerminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->m_nChannel, out);
	
	/* Verify terminal */
	if(!dpvt->m_pTerminal)
	{
		Error("EL40XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}
	free(out);
	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;
	
	/* Lock mutex for IO */
	int status = dpvt->m_pTerminal->m_pDevice->Lock();
	/* Verify it's error free */
	if(status)
	{
		Error("EL40XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_EMUTEXTIMEOUT));
		return 1;
	}


	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->m_pTerminal->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);
	
	dpvt->m_pDevice->Unlock();

	/* Verify terminal ID */
	if(termid != dpvt->m_pTerminal->m_nTerminalID || termid == 0)
	{
		Error("EL40XX_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}
	//EL40XX_write_record(record);
	return 0;
}

static long EL40XX_write_record(void* record)
{
	aoRecord* pRecord = (aoRecord*)record;
	CALLBACK* callback = (CALLBACK*)malloc(sizeof(CALLBACK));
	*callback = *(CALLBACK*)EL40XX_WriteCallback;
	callbackSetCallback(EL40XX_WriteCallback, callback);
	callbackSetUser(pRecord, callback);
	callbackSetPriority(priorityHigh, callback);
	callbackRequest(callback);
	return 0;
}

static long EL40XX_linconv(void* precord, int after)
{
	if(!after) return 0;
	aoRecord* pRecord = (aoRecord*)precord;
	pRecord->eslo = (pRecord->eguf - pRecord->egul) / 0x7FFF;
	pRecord->roff = 0; /* NO offset is needed */
	return 0;
}

