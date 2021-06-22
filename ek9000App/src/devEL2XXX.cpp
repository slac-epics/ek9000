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

struct {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write_record;
} devEL20XX = {
	5,	  (DEVSUPFUN)EL20XX_dev_report,	  (DEVSUPFUN)EL20XX_init, (DEVSUPFUN)EL20XX_init_record,
	NULL, (DEVSUPFUN)EL20XX_write_record,
};

epicsExportAddress(dset, devEL20XX);

struct EL20XXDpvt_t {
	int channel;
	devEK9000Terminal* terminal;
	devEK9000* device;
};

static void EL20XX_WriteCallback(CALLBACK* callback) {
	boRecord* pRecord;
	void* record;
	callbackGetUser(record, callback);
	pRecord = (boRecord*)record;
	EL20XXDpvt_t* dpvt = (EL20XXDpvt_t*)pRecord->dpvt;
	free(callback);

	/* Check for invalid */
	if (!dpvt->terminal)
		return;

	/* Lock & verify mutex */
	int status = dpvt->device->Lock();

	if (status != epicsMutexLockOK) {
		DevError("EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(status));
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		pRecord->pact = 0;
		return;
	}

	uint16_t buf = pRecord->val;

	/* Write to buffer */
	status = dpvt->terminal->doEK9000IO(MODBUS_WRITE_MULTIPLE_COILS,
										dpvt->terminal->m_outputStart + (dpvt->channel - 1), &buf, 1);

	dpvt->device->Unlock();

	/* Processing done */
	pRecord->pact = FALSE;
	pRecord->rbv = pRecord->val;
	pRecord->udf = FALSE;
	/* check for errors... */
	if (status) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			DevError("EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			return;
		}
		DevError("EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(status));
		return;
	}
}

static long EL20XX_dev_report(int) {
	return 0;
}

static long EL20XX_init(int) {
	return 0;
}

static long EL20XX_init_record(void* precord) {
	boRecord* pRecord = (boRecord*)precord;
	pRecord->dpvt = malloc(sizeof(EL20XXDpvt_t));
	EL20XXDpvt_t* dpvt = (EL20XXDpvt_t*)pRecord->dpvt;

	/* Grab terminal info */
	char* recname = NULL;
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(pRecord->name, dpvt->channel, recname);

	/* Verify terminal */
	if (dpvt->terminal == NULL) {
		Error("EL20XX_init_record(): Unable to find terminal for %s\n", pRecord->name);
		return 1;
	}
	free(recname);
	/* Just another param reference */
	dpvt->device = dpvt->terminal->m_device;

	/* Verify the connection */
	if (!dpvt->device->VerifyConnection()) {
		Error("EL20XX_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		return 1;
	}

	/* Lock mutex for modbus */
	int status = dpvt->terminal->m_device->Lock();

	/* Check mutex status */
	if (status != epicsMutexLockOK) {
		Error("EL20XX_init_record(): %s\n", devEK9000::ErrorToString(EK_EMUTEXTIMEOUT));
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);

	/* Verify terminal ID */
	if (termid == 0 || termid != dpvt->terminal->m_terminalId) {
		Error("EL20XX_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		dpvt->device->Unlock();
		return 1;
	}

	dpvt->device->Unlock();
	return 0;
}

static long EL20XX_write_record(void* precord) {
	util::setupCallback(precord, EL20XX_WriteCallback);
	return 0;
}
