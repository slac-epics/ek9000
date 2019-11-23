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
// Name: devEL2XXX.cpp
// Purpose: Device support for EL2xxx modules (digital out)
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
#include <mbboRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <alarm.h>
#include <alarmString.h> 
#include <recGbl.h>
#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "devEK9000.h"

//======================================================//
//
// EL20XX device support
//
//======================================================//
static void EL20XX_WriteCallback(CALLBACK* callback);
static long EL20XX_dev_report(int interest);
static long EL20XX_init(int after);
static long EL20XX_init_record(void* precord);
static long EL20XX_write_record(void* precord);

struct
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write_record;
} devEL20XX = {
	5,
	(DEVSUPFUN)EL20XX_dev_report,
	(DEVSUPFUN)EL20XX_init,
	(DEVSUPFUN)EL20XX_init_record,
	NULL,
	(DEVSUPFUN)EL20XX_write_record,
};

epicsExportAddress(dset, devEL20XX);

struct SEL20XXSupportData
{
	int m_nChannel;
	CTerminal* m_pTerminal;
	CEK9000Device* m_pDevice;
};

static void EL20XX_WriteCallback(CALLBACK* callback)
{
	boRecord* pRecord;
	void* record;
	callbackGetUser(record, callback);
	pRecord = (boRecord*)record;
	SEL20XXSupportData* dpvt = (SEL20XXSupportData*)pRecord->dpvt;
	free(callback);
	
	/* Check for invalid */
	if(!dpvt->m_pTerminal)
		return;

	/* Lock & verify mutex */
	int status = dpvt->m_pDevice->Lock();
	
	if(status != epicsMutexLockOK)
	{
		DevError("EL20XX_WriteCallback(): %s\n", CEK9000Device::ErrorToString(status));
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		pRecord->pact = 0;
		return;
	}

	uint16_t buf = pRecord->val;

	/* Write to buffer */
	status = dpvt->m_pTerminal->doEK9000IO(MODBUS_WRITE_MULTIPLE_COILS, 
		dpvt->m_pTerminal->m_nOutputStart + (dpvt->m_nChannel-2), &buf, 1);

	dpvt->m_pDevice->Unlock();

	/* Processing done */
	pRecord->pact = FALSE;
	pRecord->rbv = pRecord->val;
	pRecord->udf = FALSE;
	/* check for errors... */
	if(status)
	{
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if(status > 0x100)
		{
			DevError("EL20XX_WriteCallback(): %s\n", CEK9000Device::ErrorToString(EK_EMODBUSERR));
			return;
		}
		DevError("EL20XX_WriteCallback(): %s\n", CEK9000Device::ErrorToString(status));
		return;
	}
}
#if 0
static void EL20XX_ReadOutputs(void* precord)
{
	boRecord* pRecord = (boRecord*)precord;
	SEL20XXSupportData* dpvt = (SEL20XXSupportData*)pRecord->dpvt;
	if(!dpvt->m_pTerminal)
		return;
	uint16_t buf = 0;
	/* Read from the device */
	dpvt->m_pTerminal->doEK9000IO(MODBUS_READ_COILS,
			dpvt->m_pTerminal->m_nOutputStart + (dpvt->m_nChannel * 2), &buf, 1);
	pRecord->rval = buf;
}
#endif

static long EL20XX_dev_report(int interest)
{
	return 0;
}

static long EL20XX_init(int after)
{
	return 0;
}

static long EL20XX_init_record(void* precord)
{
	boRecord* pRecord = (boRecord*)precord;
	pRecord->dpvt = malloc(sizeof(SEL20XXSupportData));
	SEL20XXSupportData* dpvt = (SEL20XXSupportData*)pRecord->dpvt;
	
	/* Grab terminal info */
	char* recname = NULL;
	dpvt->m_pTerminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->m_nChannel, recname);

	/* Verify terminal */
	if(dpvt->m_pTerminal == NULL)
	{
		Error("EL20XX_init_record(): Unable to find terminal for %s\n", pRecord->name);
		return 1;
	}
	free(recname);
	/* Just another param reference */
	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;
	
	/* Verify the connection */
	if(!dpvt->m_pDevice->VerifyConnection())
	{
		Error("EL20XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		return 1;
	}

	/* Lock mutex for modbus */
	int status = dpvt->m_pTerminal->m_pDevice->Lock();

	/* Check mutex status */
	if(status != epicsMutexLockOK)
	{
		Error("EL20XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_EMUTEXTIMEOUT));
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);

	/* Verify terminal ID */
	if(termid == 0 || termid != dpvt->m_pTerminal->m_nTerminalID)
	{
		Error("EL20XX_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		dpvt->m_pDevice->Unlock();
		return 1;
	}

	dpvt->m_pDevice->Unlock();
	//EL20XX_write_record(precord);
	return 0;
}

static long EL20XX_write_record(void* precord)
{
	boRecord* pRecord = (boRecord*)precord;
	SEL20XXSupportData* dat = (SEL20XXSupportData*)pRecord->dpvt;
	CALLBACK* callback = (CALLBACK*)malloc(sizeof(CALLBACK));
	*callback = *(CALLBACK*)EL20XX_WriteCallback;
	callbackSetUser(pRecord, callback);
	callbackSetPriority(priorityHigh, callback);
	callbackSetCallback(EL20XX_WriteCallback, callback);
	callbackRequest(callback);
	//dat->m_pDevice->QueueCallback(EL20XX_WriteCallback, precord);
	return 0;
}
