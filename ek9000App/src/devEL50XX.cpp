/*
 *
 * EL50XX device support (encoder interfaces)
 *
 */ 
#include <epicsPrint.h>
#include <epicsStdio.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <alarm.h>
#include <recGbl.h>

#include <longinRecord.h>

#include "devEK9000.h"
#include "devEL50XX.h"

struct SEL50XXSupportData
{
	uint32_t tid;
	CTerminal* pterm;
	CEK9000Device* pcoupler;
	longinRecord* precord;
	union
	{
		SEL5001Output el5001_output;
		SEL5002Output el5002_output;
	};
};

static void el50xx_read_callback(CALLBACK* callback);
static long el50xx_dev_report(int lvl);
static long el50xx_init(int after);
static long el50xx_init_record(void* precord);
static long el50xx_read_record(void* precord);

struct 
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
} devEL50XX = {
	5,
	(DEVSUPFUN)el50xx_dev_report,
	(DEVSUPFUN)el50xx_init,
	el50xx_init_record,
	nullptr,
	el50xx_read_record,
};

extern "C" {
	epicsExportAddress(dset, devEL50XX);
}

static void el50xx_read_callback(CALLBACK* callback)
{
	void* usr;
	callbackGetUser(usr, callback);
	SEL50XXSupportData* dpvt = (SEL50XXSupportData*)usr;

	if(!dpvt || !dpvt->pterm || !dpvt->pcoupler)
		return;

	if(!dpvt->pcoupler->VerifyConnection())
	{
		recGblSetSevr((longinRecord*)dpvt->precord, COMM_ALARM, INVALID_ALARM);
		free(callback);
		return;
	}

	/* Read into a buffer that's plenty big enough for any terminal type */
	uint16_t data[32];
	dpvt->pcoupler->doEK9000IO(0, dpvt->pterm->m_nInputStart, 
		STRUCT_SIZE_TO_MODBUS_SIZE(dpvt->pterm->m_nInputSize), data);

	/* Handle individual terminal pdo types */
	switch(dpvt->tid)
	{
		case 5001:
		{
			SEL5001Output* output = reinterpret_cast<SEL5001Output*>(data);
			if(output->data_error || output->sync_err)
				recGblSetSevr(dpvt->precord, READ_ALARM, INVALID_ALARM);
			if(output->frame_error)
				recGblSetSevr(dpvt->precord, READ_ALARM, MAJOR_ALARM);
			dpvt->precord->val = output->encoder_value;
			break;
		}
		case 5002:
		{
			SEL5002Output* output = reinterpret_cast<SEL5002Output*>(data);
			if(output->data_error)
				recGblSetSevr(dpvt->precord, READ_ALARM, INVALID_ALARM);
			if(output->frame_error)
				recGblSetSevr(dpvt->precord, COMM_ALARM, MAJOR_ALARM);
			dpvt->precord->val = output->encoder_value;
			break;
		}
		default:
		{
			/* Raise invalid alarm if we don't have a tid */
			recGblSetSevr(dpvt->precord, READ_ALARM, INVALID_ALARM);
		}
	}
	free(callback);

}

static long el50xx_dev_report(int lvl)
{

}

static long el50xx_init(int after)
{

}

static long el50xx_init_record(void* precord)
{
	longinRecord* record = static_cast<longinRecord*>(precord);
	record->dpvt = calloc(1, sizeof(SEL50XXSupportData));
	SEL50XXSupportData* dpvt = static_cast<SEL50XXSupportData*>(record->dpvt);

	/* Get the terminal */
	char* recname = NULL;
	int channel = 0;
	dpvt->pterm = CTerminal::ProcessRecordName(record->name, channel, recname);
	if(!dpvt->pterm)
	{
		Error("EL50XX_init_record(): Unable to find terminal for record %s\n", record->name);
		return 1;
	}
	free(recname);
	
	dpvt->pcoupler = dpvt->pterm->m_pDevice;
	dpvt->pcoupler->Lock();

	/* Check connection to terminal */
	if(!dpvt->pcoupler->VerifyConnection())
	{
		Error("EL50XX_init_record(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		dpvt->pcoupler->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->pterm->m_pDevice->ReadTerminalID(dpvt->pterm->m_nTerminalIndex, termid);
	dpvt->pcoupler->Unlock();
	dpvt->tid = termid;

	if(termid != dpvt->pterm->m_nTerminalID || termid == 0)
	{
		Error("EL50XX_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), record->name, termid);
		return 1;
	}

	return 0;
}

static long el50xx_read_record(void* precord)
{
	longinRecord* prec = (longinRecord*)precord;
	SEL50XXSupportData* dpvt = (SEL50XXSupportData*)prec->dpvt;
	dpvt->precord = static_cast<longinRecord*>(precord);

	/* Indicate processing */
	prec->pact = 1;

	CALLBACK* callback = (CALLBACK*)calloc(1, sizeof(CALLBACK));
	callbackSetUser(precord, callback);
	callbackSetPriority(priorityMedium, callback);
	callbackRequest(callback);
	return 0;
}


static long el5042_dev_report(int lvl);
static long el5042_init_record(void* prec);
static long el5042_init(int after);
static long el5042_read_record(void* prec);
static void el5042_read_callback(CALLBACK* callback);

struct 
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
} devEL5042 = {
	5,
	(DEVSUPFUN)el5042_dev_report,
	(DEVSUPFUN)el5042_init,
	el5042_init_record,
	nullptr,
	el5042_read_record,
};

extern "C" {
	epicsExportAddress(dset, devEL5042);
}

struct SEL5042SupportData
{
	CTerminal* pterm;
	CEK9000Device* pcoupler;
};

/*
-------------------------------------
Report on all EL5042 devices
-------------------------------------
*/
static long el5042_dev_report(int lvl)
{

}


/*
-------------------------------------
Initialize the specified record
-------------------------------------
*/
static long el5042_init_record(void* prec)
{
	longinRecord* record = static_cast<longinRecord*>(prec);
	record->dpvt = calloc(1, sizeof(SEL50XXSupportData));
	SEL5042SupportData* dpvt = static_cast<SEL5042SupportData*>(record->dpvt);

	/* Get the terminal */
	char* recname = NULL;
	int channel = 0;
	dpvt->pterm = CTerminal::ProcessRecordName(record->name, channel, recname);
	if(!dpvt->pterm)
	{
		Error("EL5042_init_record(): Unable to find terminal for record %s\n", record->name);
		return 1;
	}
	free(recname);
	
	dpvt->pcoupler = dpvt->pterm->m_pDevice;
	dpvt->pcoupler->Lock();

	/* Check connection to terminal */
	if(!dpvt->pcoupler->VerifyConnection())
	{
		Error("EL5042_init_record(): %s\n", CEK9000Device::ErrorToString(EK_ENOCONN));
		dpvt->pcoupler->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->pterm->m_pDevice->ReadTerminalID(dpvt->pterm->m_nTerminalIndex, termid);
	dpvt->pcoupler->Unlock();
	if(termid != dpvt->pterm->m_nTerminalID || termid == 0)
	{
		Error("EL5042_init_record(): %s: %s != %u\n", CEK9000Device::ErrorToString(EK_ETERMIDMIS), record->name, termid);
		return 1;
	}

	return 0;
}


/*
-------------------------------------
Initialize the device support module
-------------------------------------
*/
static long el5042_init(int after)
{

}


/*
-------------------------------------
Called to read the specified record
-------------------------------------
*/
static long el5042_read_record(void* prec)
{
	
}


/*
-------------------------------------
Callback queued by read_record to actually
read the record
-------------------------------------
*/
static void el5042_read_callback(CALLBACK* callback)
{

}