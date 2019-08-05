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

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>

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
	callbackGetUser(record, callback);
	aiRecord* pRecord = (aiRecord*)record;
	SEL30XXSupportData* dpvt = (SEL30XXSupportData*)pRecord->dpvt;

	/* Lock mutex */
	int status = dpvt->m_pTerminal->m_pDevice->Lock();

	if(status != epicsMutexLockOK)
	{
		dpvt->m_pDevice->ReportError(EK_EMUTEXTIMEOUT, "EL30XX_ReadCallback");
		return;
	}

	/* read analog input, 4 bytes each, first 16 bytes is the actual adc value */
	uint16_t buf[2];
	status = dpvt->m_pTerminal->doEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->m_pTerminal->m_nInputStart +
		(dpvt->m_nChannel * 2), buf, 2);

	/* Set props */
	pRecord->rval = buf[0];
	pRecord->pact = 0;

	/* Check for error */
	if(status)
	{
		dpvt->m_pDevice->Unlock();
		if(status > 0x100)
		{
			dpvt->m_pDevice->ReportError(EK_EMODBUSERR, "EL30XX_ReadCallback");			
			return;
		}
		dpvt->m_pDevice->ReportError(status, "EL30XX_ReadCallback");
		return;
	}

	/* Unlock device */
	dpvt->m_pTerminal->m_pDevice->Unlock();

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
	aiRecord* pRecord = (aiRecord*)precord;
	pRecord->dpvt = calloc(1, sizeof(SEL30XXSupportData));
	SEL30XXSupportData* dpvt = (SEL30XXSupportData*)pRecord->dpvt;

	/* Get the terminal */
	char* recname = 0;
	dpvt->m_pTerminal = CTerminal::ProcessRecordName(pRecord->name, dpvt->m_nChannel, recname);
	if(!dpvt->m_pTerminal)
	{
		dbgprintf("Failed to create terminal.")
		return 1;
	}
	free(recname);

	dpvt->m_pDevice = dpvt->m_pTerminal->m_pDevice;
	dpvt->m_pDevice->Lock();

	/* Check connection to terminal */
	if(!dpvt->m_pTerminal->m_pDevice->VerifyConnection())
	{
		dpvt->m_pDevice->ReportError(EK_ENOCONN);
		free(dpvt);
		dpvt->m_pDevice->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->m_pTerminal->m_pDevice->ReadTerminalID(dpvt->m_pTerminal->m_nTerminalIndex, termid);

	if(termid == 0)
	{
		epicsStdoutPrintf("Error while reading terminal id for %s.", pRecord->name);
		free(dpvt);
		dpvt->m_pDevice->Unlock();
		return 1;
	}
	if(termid != dpvt->m_pTerminal->m_nTerminalID)
	{
		epicsStdoutPrintf("Error: Slave #%u has configured ID %u, but the coupler reports it has ID %u.",
			dpvt->m_pTerminal->m_nTerminalIndex, dpvt->m_pTerminal->m_nTerminalID, termid);
		free(dpvt);
		dpvt->m_pDevice->Unlock();
		return 1;
	}

	dpvt->m_pDevice->Unlock();

	//EL30XX_read_record(precord);

	return 0;
}

static long EL30XX_read_record(void *precord)
{
	aiRecord* pRecord = (aiRecord*)precord;
	SEL30XXSupportData* dpvt = (SEL30XXSupportData*)pRecord->dpvt;

	/* indicate processing */
	pRecord->pact = 1;

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
	aiRecord* pRecord = (aiRecord*)precord;
	/* Max range is 32767 */
	pRecord->eslo = (pRecord->eguf - pRecord->egul) / 32767;
	pRecord->roff = 0x0;
	return 0;
}


#if 0
//======================================================//
//
// EL30XX settings record type
//
//======================================================//
static long recEL30XX_report(void* precord);
static long recEL30XX_initialize();
static long recEL30XX_init_record(void* precord, int pass);
static long recEL30XX_process(void* precord);
static long recEL30XX_cvt_dbaddr(dbAddr* paddr);
static long recEL30XX_get_arr_info(dbAddr* paddr, long* nelems, long* offset);
static long recEL30XX_get_units(dbAddr* paddr, char* punits);
static long recEL30XX_get_precision(dbAddr* paddr, long* precision);
static long recEL30XX_get_enum_str(dbAddr* paddr, char* p);
static long recEL30XX_get_enum_strs(dbAddr* paddr, dbr_enumStrs* strs);
static long recEL30XX_put_enum_str(dbAddr* paddr, char* p);
static long recEL30XX_get_graphic_double(dbAddr* paddr, dbr_grDouble* dbl);
static long recEL30XX_get_control_double(dbAddr* paddr, dbr_ctrlDouble* dbl);
static long recEL30XX_get_alarm_double(dbAddr* paddr, dbr_alDouble* dbl);

#endif