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
#include <devSup.h>
#include <alarm.h>
#include <mbboDirectRecord.h>
#include <callback.h>
#include <recGbl.h>
#include <drvModbusAsyn.h>

#include <stddef.h>
#include <stdint.h>

#include "devEK9000.h"

//======================================================//
//
// EL20XX device support
//
//======================================================//

static inline uint16_t get_nobt(boRecord* record) {
	return 1;
}
static inline uint16_t get_nobt(mbboDirectRecord* record) {
	return record->nobt;
}

template <class RecordT> static void EL20XX_WriteCallback(CALLBACK* callback) {
	RecordT* pRecord;
	void* record;
	callbackGetUser(record, callback);
	pRecord = (RecordT*)record;
	TerminalDpvt_t* dpvt = (TerminalDpvt_t*)pRecord->dpvt;
	free(callback);

	/* Check for invalid */
	if (!dpvt->pterm) {
		pRecord->pact = FALSE;
		return;
	}

	/* Lock & verify mutex */
	DeviceLock lock(dpvt->pdrv);

	if (!lock.valid()) {
		LOG_ERROR(dpvt->pdrv, "failed to obtain device lock\n");
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		pRecord->pact = FALSE;
		return;
	}

	const bool mbbo = util::is_same<mbboDirectRecord, RecordT>::value;

	uint16_t buf[32];
	const uint16_t length = get_nobt(pRecord); // Will be 1 for bo, nobt for mbboDirect
	if (mbbo) {
		// Inflate packed bit data into the input buffer
		for (int i = 0; i < length; ++i) {
			buf[i] = (pRecord->rval & (1 << i)) >> i;
		}
	}
	else {
		buf[0] = pRecord->val;
	}

	const uint16_t addr =
		mbbo ? dpvt->pterm->m_outputStart - 1 : dpvt->pterm->m_outputStart + (dpvt->channel - 2);

	/* Write to buffer */
	/** The logic here: channel - 1 for a 0-based index, and subtract another 1 because modbus coils start at 0, and
	 * inputStart is 1-based **/
	int status = dpvt->pterm->doEK9000IO(MODBUS_WRITE_MULTIPLE_COILS, addr, buf, length);

	lock.unlock();

	/* check for errors... */
	if (status) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			LOG_WARNING(dpvt->pdrv, "EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			pRecord->pact = FALSE;
			return;
		}
		LOG_WARNING(dpvt->pdrv, "EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(status));
		pRecord->pact = FALSE;
		return;
	}

	/* OK, we've written a value, everything looks good.  We need to reprocess this! */
	struct typed_rset* prset = (struct typed_rset*)(pRecord->rset);
	dbScanLock((struct dbCommon*)pRecord);
	pRecord->rbv = pRecord->val;
	pRecord->udf = FALSE;
	(*prset->process)((struct dbCommon*)pRecord); /* This will set PACT false! */
	dbScanUnlock((struct dbCommon*)pRecord);
}

static long EL20XX_dev_report(int) {
	return 0;
}

static long EL20XX_init(int) {
	return 0;
}

static inline void type_specific_setup(boRecord* record, int16_t numbits){};
static inline void type_specific_setup(mbboDirectRecord* record, int16_t numbits) {
	record->nobt = numbits;
	record->mask = (1 << numbits) - 1;
	record->shft = 0;
}

template <class RecordT> static long EL20XX_init_record(void* precord) {
	RecordT* pRecord = (RecordT*)precord;
	pRecord->dpvt = util::allocDpvt();
	TerminalDpvt_t* dpvt = (TerminalDpvt_t*)pRecord->dpvt;

	const bool mbbo = util::is_same<RecordT, mbboDirectRecord>::value;

	/* Grab terminal info */
	if (!util::setupCommonDpvt<RecordT>(pRecord, *dpvt)) {
		LOG_ERROR(dpvt->pdrv, "Unable to setup dpvt for %s\n", pRecord->name);
		return 1;
	}

	type_specific_setup(pRecord, dpvt->pterm->m_outputSize);

	/* Verify the connection */
	if (!dpvt->pdrv->VerifyConnection()) {
		LOG_ERROR(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_ENOCONN));
		return 1;
	}

	/* Lock mutex for modbus */
	DeviceLock lock(dpvt->pdrv);

	/* Check mutex status */
	if (!lock.valid()) {
		LOG_ERROR(dpvt->pdrv, "unable to obtain device lock\n");
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->pdrv->ReadTerminalID(dpvt->pterm->m_terminalIndex, termid);

	/* Verify terminal ID */
	if (termid == 0 || termid != dpvt->pterm->m_terminalId) {
		LOG_ERROR(dpvt->pdrv, "%s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name,
					termid);
		return 1;
	}
	return 0;
}

template <class T> static long EL20XX_write_record(void* precord) {
	T* prec = (T*)precord;
	if (prec->pact)
		prec->pact = FALSE;
	else {
		prec->pact = TRUE;
		util::setupCallback(precord, EL20XX_WriteCallback<T>);
	}
	return 0;
}

struct devEL20XX_t {
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
	(DEVSUPFUN)EL20XX_init_record<boRecord>,
	NULL,
	(DEVSUPFUN)EL20XX_write_record<boRecord>,
};

epicsExportAddress(dset, devEL20XX);

struct {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write_record;
} devEL20XX_mbboDirect = {
	5,
	(DEVSUPFUN)EL20XX_dev_report,
	(DEVSUPFUN)EL20XX_init,
	(DEVSUPFUN)EL20XX_init_record<mbboDirectRecord>,
	NULL,
	(DEVSUPFUN)EL20XX_write_record<mbboDirectRecord>,
};

epicsExportAddress(dset, devEL20XX_mbboDirect);
