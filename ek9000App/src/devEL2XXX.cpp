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

struct EL20XXDpvt_t {
	int channel;
	devEK9000Terminal* terminal;
	devEK9000* device;
};

static inline uint16_t get_nobt(boRecord* record) { return 1; }
static inline uint16_t get_nobt(mbboDirectRecord* record) { return record->nobt; }

template<class RecordT>
static void EL20XX_WriteCallback(CALLBACK* callback) {
	RecordT* pRecord;
	void* record;
	callbackGetUser(record, callback);
	pRecord = (RecordT*)record;
	EL20XXDpvt_t* dpvt = (EL20XXDpvt_t*)pRecord->dpvt;
	free(callback);

	/* Check for invalid */
	if (!dpvt->terminal) {
		pRecord->pact = FALSE;
		return;
	}

	/* Lock & verify mutex */
	int status = dpvt->device->Lock();

	if (status) {
		DevError("EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(status));
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		pRecord->pact = FALSE;
		return;
	}

	const bool mbbo = std::is_same<mbboDirectRecord, RecordT>::value;

	uint16_t buf[32];
	const uint16_t length = get_nobt(pRecord); // Will be 1 for bo, nobt for mbboDirect
	if (mbbo) {
		// Inflate packed bit data into the input buffer
		for (int i = 0; i < length; ++i) {
			buf[i] = (pRecord->rval & (1<<i)) >> i;
		}
	}
	else {
		buf[0] = pRecord->val;
	}
	
	const uint16_t addr = 
		mbbo ?
		dpvt->terminal->m_outputStart - 1 :
		dpvt->terminal->m_outputStart + (dpvt->channel - 2);

	/* Write to buffer */
	/** The logic here: channel - 1 for a 0-based index, and subtract another 1 because modbus coils start at 0, and
	 * inputStart is 1-based **/
	status = dpvt->terminal->doEK9000IO(MODBUS_WRITE_MULTIPLE_COILS, addr, buf, length);

	dpvt->device->Unlock();

	/* check for errors... */
	if (status) {
		recGblSetSevr(pRecord, WRITE_ALARM, INVALID_ALARM);
		if (status > 0x100) {
			DevError("EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(EK_EMODBUSERR));
			pRecord->pact = FALSE;
			return;
		}
		DevError("EL20XX_WriteCallback(): %s\n", devEK9000::ErrorToString(status));
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

static inline void type_specific_setup(boRecord* record, int16_t numbits) {};
static inline void type_specific_setup(mbboDirectRecord* record, int16_t numbits) {
	record->nobt = numbits;
	record->mask = (1<<numbits) - 1;
	record->shft = 0;
}

template<class RecordT>
static long EL20XX_init_record(void* precord) {
	RecordT* pRecord = (RecordT*)precord;
	pRecord->dpvt = calloc(1, sizeof(EL20XXDpvt_t));
	EL20XXDpvt_t* dpvt = (EL20XXDpvt_t*)pRecord->dpvt;

	const bool mbbo = std::is_same<RecordT, mbboDirectRecord>::value;

	/* Grab terminal info */
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(pRecord->name, mbbo ? nullptr : &dpvt->channel);

	/* Verify terminal */
	if (dpvt->terminal == NULL) {
		util::Error("EL20XX_init_record(): Unable to find terminal for %s\n", pRecord->name);
		return 1;
	}
	
	type_specific_setup(pRecord, dpvt->terminal->m_outputSize);

	/* Just another param reference */
	dpvt->device = dpvt->terminal->m_device;

	/* Verify the connection */
	if (!dpvt->device->VerifyConnection()) {
		util::Error("EL20XX_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		return 1;
	}

	/* Lock mutex for modbus */
	int status = dpvt->terminal->m_device->Lock();

	/* Check mutex status */
	if (status != epicsMutexLockOK) {
		util::Error("EL20XX_init_record(): %s\n", devEK9000::ErrorToString(EK_EMUTEXTIMEOUT));
		return 1;
	}

	/* Read terminal ID */
	uint16_t termid = 0;
	dpvt->device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);

	/* Verify terminal ID */
	if (termid == 0 || termid != dpvt->terminal->m_terminalId) {
		util::Error("EL20XX_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name,
					termid);
		dpvt->device->Unlock();
		return 1;
	}

	dpvt->device->Unlock();
	return 0;
}

template<class T>
static long EL20XX_write_record(void* precord) {
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
	5,	  (DEVSUPFUN)EL20XX_dev_report,	  (DEVSUPFUN)EL20XX_init, (DEVSUPFUN)EL20XX_init_record<boRecord>,
	NULL, (DEVSUPFUN)EL20XX_write_record<boRecord>,
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
	5,	  (DEVSUPFUN)EL20XX_dev_report,	  (DEVSUPFUN)EL20XX_init, (DEVSUPFUN)EL20XX_init_record<mbboDirectRecord>,
	NULL, (DEVSUPFUN)EL20XX_write_record<mbboDirectRecord>,
};

epicsExportAddress(dset, devEL20XX_mbboDirect);