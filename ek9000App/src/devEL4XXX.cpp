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

struct EL40XXDpvt_t {
	int channel;
	devEK9000Terminal* terminal;
	devEK9000* device;
	/* Standard or compact PDO used */
	bool compactPDO;
};

static long EL40XX_dev_report(int after);
static long EL40XX_init(int after);
static long EL40XX_init_record(void* record);
static long EL40XX_write_record(void* record);
static long EL40XX_linconv(void* precord, int after);

struct {
	long num;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN ioint_info;
	DEVSUPFUN write_record;
	DEVSUPFUN linconv;
} devEL40XX = {
	6,	  (DEVSUPFUN)EL40XX_dev_report,	  (DEVSUPFUN)EL40XX_init,	 (DEVSUPFUN)EL40XX_init_record,
	NULL, (DEVSUPFUN)EL40XX_write_record, (DEVSUPFUN)EL40XX_linconv,
};

epicsExportAddress(dset, devEL40XX);

static void EL40XX_WriteCallback(CALLBACK* callback) {
	void* record = NULL;
	callbackGetUser(record, callback);
	aoRecord* pRecord = (aoRecord*)record;
	EL40XXDpvt_t* dpvt = (EL40XXDpvt_t*)pRecord->dpvt;
	free(callback);

	/* Check for invalid */
	if (!dpvt->terminal)
		return;

	/* Verify connection */
	if (!dpvt->terminal->m_device->VerifyConnection()) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		DevError("EL40XX_WriteCallback(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		pRecord->pact = 0;
		return;
	}

	/* Lock the mutex */
	dpvt->terminal->m_device->Lock();

	/* Set buffer & do write */
	uint16_t buf = (int16_t)pRecord->rval;
	int status = dpvt->terminal->doEK9000IO(MODBUS_WRITE_MULTIPLE_REGISTERS,
											dpvt->terminal->m_outputStart + (dpvt->channel - 1), &buf, 1);

	/* Unlock mutex */
	dpvt->terminal->m_device->Unlock();

	/* No more processing */
	pRecord->pact = FALSE;
	pRecord->udf = FALSE;
	/* Check error */
	if (status) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			DevError("EL40XX_WriteCallback(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return;
		} else {
			DevError("EL40XX_WriteCallback(): %s\n", devEK9000::ErrorToString(status));
		}
		return;
	}
}

static long EL40XX_dev_report(int) {
	return 0;
}

static long EL40XX_init(int) {
	return 0;
}

static long EL40XX_init_record(void* record) {
	aoRecord* pRecord = (aoRecord*)record;
	pRecord->dpvt = calloc(1, sizeof(EL40XXDpvt_t));
	EL40XXDpvt_t* dpvt = (EL40XXDpvt_t*)pRecord->dpvt;

	/* Find record name */
	char* out = NULL;
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(pRecord->name, dpvt->channel, out);

	/* Verify terminal */
	if (!dpvt->terminal) {
		util::Error("EL40XX_init_record(): Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}
	free(out);
	dpvt->device = dpvt->terminal->m_device;

	/* Lock mutex for IO */
	int status = dpvt->terminal->m_device->Lock();
	/* Verify it's error free */
	if (status) {
		util::Error("EL40XX_init_record(): %s\n", devEK9000::ErrorToString(EK_EMUTEXTIMEOUT));
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->terminal->m_device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);

	dpvt->device->Unlock();

	/* Verify terminal ID */
	if (termid != dpvt->terminal->m_terminalId || termid == 0) {
		util::Error("EL40XX_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}

	return 0;
}

static long EL40XX_write_record(void* record) {
	util::setupCallback(record, EL40XX_WriteCallback);
	return 0;
}

static long EL40XX_linconv(void*, int) {
	return 0;
}
