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

	/* Verify connection */
	if(!dpvt->m_pTerminal->m_pDevice->VerifyConnection())
	{
		dbgprintf("Device not connected");
		pRecord->pact = 0;
		return;
	}

	/* Lock the mutex */
	dpvt->m_pTerminal->m_pDevice->Lock();
	
	/* Set buffer & do write */
	uint16_t buf = pRecord->rval;
	int status = dpvt->m_pTerminal->doEK9000IO(MODBUS_WRITE_MULTIPLE_REGISTERS, dpvt->m_pTerminal->m_nOutputStart + 
		(dpvt->m_nChannel-1), &buf, 1);
	
	/* Unlock mutex */
	dpvt->m_pTerminal->m_pDevice->Unlock();

	/* No more processing */
	pRecord->pact = 0;

	/* Check error */
	if(status)
	{
		if(status > 0x100)
		{
			dpvt->m_pDevice->ReportError(EK_EMODBUSERR);
			return;
		}
		else
		{
			dpvt->m_pDevice->ReportError(status);
			return;
		}
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
		dbgprintf("Unable to create terminal %s.", pRecord->name);
		return 1;
	}
	free(out);
	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;
	
	/* Lock mutex for IO */
	int status = dpvt->m_pTerminal->m_pDevice->Lock();
	/* Verify it's error free */
	if(status)
	{
		dpvt->m_pDevice->ReportError(EK_EMUTEXTIMEOUT, "EL40XX_init_record");
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->m_pTerminal->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);

	/* Verify terminal ID */
	if(termid == 0)
	{
		dpvt->m_pDevice->ReportError(EK_EBADTERM, "EL40XX_init_record");
		return 1;
	}
	if(termid != dpvt->m_pTerminal->m_nTerminalID)
	{
		dpvt->m_pDevice->ReportError(EK_ETERMIDMIS, "EL40XX_init_record");
		return 1;
	}
	
	/* Unlock mutex */
	dpvt->m_pTerminal->m_pDevice->Unlock();
	
	/* Do a record write to propagate values */
	EL40XX_write_record(pRecord);
	
	return 0;
}

static long EL40XX_write_record(void* record)
{
	aoRecord* pRecord = (aoRecord*)record;
	SEL40XXSupportData* dpvt = (SEL40XXSupportData*)pRecord->dpvt;
	CALLBACK* callback = (CALLBACK*)malloc(sizeof(CALLBACK));
	*callback = *(CALLBACK*)EL40XX_WriteCallback;
	callbackSetCallback(EL40XX_WriteCallback, callback);
	callbackSetUser(pRecord, callback);
	callbackSetPriority(priorityHigh, callback);
	pRecord->pact = 1;
	callbackRequest(callback);
	return 0;
}

static long EL40XX_linconv(void* precord, int after)
{
	aoRecord* pRecord = (aoRecord*)precord;
	/* Output is bit shifted 3 bits such that the max value will be 32767 */
	pRecord->eslo = (pRecord->eguf - pRecord->egul) / 0x7FFF;
	pRecord->roff = 0; /* NO offset is needed */
	return 0;
}