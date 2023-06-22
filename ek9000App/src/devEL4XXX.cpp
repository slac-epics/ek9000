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
#include <devSup.h>
#include <alarm.h>
#include <aoRecord.h>
#include <callback.h>
#include <recGbl.h>

#include <drvModbusAsyn.h>

#include <stddef.h>
#include <stdint.h>

#include "ekUtil.h"
#include "devEK9000.h"

#include "terminal_types.g.h"

static long EL40XX_dev_report(int after);
static long EL40XX_init(int after);
static long EL40XX_init_record(void* record);
static long EL40XX_write_record(void* record);
static long EL40XX_linconv(void* precord, int after);

struct devEL40XX_t {
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

DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4001);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4002);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4004);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4008);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4011);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4012);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4014);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4018);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4021);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4022);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4024);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4028);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4031);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4032);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4034);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4038);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4102);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4104);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4112);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4114);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4122);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4132);
DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(uint16_t, EL4134);

static void EL40XX_WriteCallback(CALLBACK* callback) {
	void* record = NULL;
	callbackGetUser(record, callback);
	aoRecord* pRecord = (aoRecord*)record;
	TerminalDpvt_t* dpvt = (TerminalDpvt_t*)pRecord->dpvt;
	free(callback);

	/* Check for invalid */
	if (!dpvt->pterm) {
		pRecord->pact = FALSE;
		return;
	}

	/* Verify connection */
	if (!dpvt->pterm->m_device->VerifyConnection()) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		LOG_ERROR(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_ENOCONN));
		pRecord->pact = FALSE;
		return;
	}

	/* Lock the mutex */
	DeviceLock lock(dpvt->pdrv);

	if (!lock.valid()) {
		LOG_ERROR(dpvt->pdrv, "unable to obtain device lock\n");
		return;
	}

	/* Set buffer & do write */
	uint16_t buf = (int16_t)pRecord->rval;
	int status = dpvt->pterm->doEK9000IO(MODBUS_WRITE_MULTIPLE_REGISTERS,
										 dpvt->pterm->m_outputStart + (dpvt->channel - 1), &buf, 1);

	/* Unlock mutex */
	lock.unlock();

	pRecord->udf = FALSE;

	/* Check error */
	if (status) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			pRecord->pact = FALSE;
			return;
		}
		else {
			LOG_WARNING(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(status));
		}
		pRecord->pact = FALSE;
		return;
	}

	/* OK, we've written a value, everything looks good.  We need to reprocess this! */
	struct typed_rset* prset = (struct typed_rset*)(pRecord->rset);
	dbScanLock((struct dbCommon*)pRecord);
	pRecord->udf = FALSE;
	(*prset->process)((struct dbCommon*)pRecord); /* This will set PACT false! */
	dbScanUnlock((struct dbCommon*)pRecord);
}

static long EL40XX_dev_report(int) {
	return 0;
}

static long EL40XX_init(int) {
	return 0;
}

static long EL40XX_init_record(void* record) {
	aoRecord* pRecord = (aoRecord*)record;
	pRecord->dpvt = util::allocDpvt();
	TerminalDpvt_t* dpvt = (TerminalDpvt_t*)pRecord->dpvt;

	/* Verify terminal */
	if (!util::setupCommonDpvt(pRecord, *dpvt)) {
		LOG_ERROR(dpvt->pdrv, "Unable to find terminal for record %s\n", pRecord->name);
		return 1;
	}

	/* Lock mutex for IO */
	DeviceLock lock(dpvt->pdrv);

	/* Verify it's error free */
	if (!lock.valid()) {
		LOG_ERROR(dpvt->pdrv, "unable to obtain device lock\n");
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->pterm->m_device->ReadTerminalID(dpvt->pterm->m_terminalIndex, termid);

	lock.unlock();

	/* Verify terminal ID */
	if (termid != dpvt->pterm->m_terminalId || termid == 0) {
		LOG_ERROR(dpvt->pdrv, "%s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}

	return 0;
}

static long EL40XX_write_record(void* record) {
	struct aoRecord* prec = (struct aoRecord*)record;
	if (prec->pact)
		prec->pact = FALSE;
	else {
		prec->pact = TRUE;
		util::setupCallback(record, EL40XX_WriteCallback);
	}
	return 0;
}

static long EL40XX_linconv(void*, int) {
	return 0;
}
