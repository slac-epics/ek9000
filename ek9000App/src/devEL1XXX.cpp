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
// Name: devEL1XXX.cpp
// Purpose: Device support for EL1xxx modules (digital in)
// Authors: Jeremy L.
// Date Created: July 6, 2019
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <devSup.h>
#include <alarm.h>
#include <biRecord.h>
#include <recGbl.h>
#include <mbbiDirectRecord.h>

#include <drvModbusAsyn.h>

#include <stddef.h>
#include <stdint.h>

#include "devEK9000.h"

static long EL10XX_dev_report(int) {
	return 0;
}

static long EL10XX_init(int) {
	return 0;
}

static inline void type_specific_setup(biRecord*, uint16_t) {
}
static inline void type_specific_setup(mbbiDirectRecord* record, uint16_t numbits) {
	record->nobt = numbits;
	record->mask = (1 << numbits) - 1;
	record->shft = 0;
}

template <class RecordT> static long EL10XX_init_record(void* precord) {
	RecordT* pRecord = (RecordT*)precord;
	pRecord->dpvt = util::allocDpvt();
	TerminalDpvt_t* dpvt = (TerminalDpvt_t*)pRecord->dpvt;

	/* Get terminal */
	const bool mbbi = util::is_same<RecordT, mbbiDirectRecord>::value;
	if (!util::setupCommonDpvt<RecordT>(pRecord, *dpvt)) {
		LOG_ERROR(dpvt->pdrv, "Unable to setup dpvt for record %s\n", pRecord->name);
		return 1;
	}

	type_specific_setup(pRecord, dpvt->pterm->m_inputSize);

	/* Lock mutex for modbus io */
	DeviceLock lock(dpvt->pdrv);

	/* Verify lock OK */
	if (!lock.valid()) {
		LOG_ERROR(dpvt->pdrv, "failed to obtain device lock\n");
		return 1;
	}

	/* Read termid */
	uint16_t termid = 0;
	dpvt->pdrv->ReadTerminalID(dpvt->pterm->m_terminalIndex, termid);

	lock.unlock();

	pRecord->udf = FALSE;

	/* Invalid term id */
	if (termid == 0 || termid != dpvt->pterm->m_terminalId) {
		LOG_ERROR(dpvt->pdrv, "%s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), pRecord->name, termid);
		return 1;
	}
	return 0;
}

static long EL10XX_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);
	if (!util::DpvtValid(dpvt))
		return 1;

	*iopvt = dpvt->pdrv->m_digital_io;
	return 0;
}

static inline void set_mbbi_rval(biRecord*, uint32_t) {
}
static inline void set_mbbi_rval(mbbiDirectRecord* record, uint32_t val) {
	record->rval = (val >> record->shft) & record->mask;
}

template <class RecordT> static long EL10XX_read_record(void* prec) {
	RecordT* pRecord = (RecordT*)prec;
	TerminalDpvt_t* dpvt = (TerminalDpvt_t*)pRecord->dpvt;

	/* Check for invalid */
	if (!util::DpvtValid(dpvt))
		return 0;

	/* Lock for modbus io */
	int status;

	const bool mbbi = util::is_same<RecordT, mbbiDirectRecord>::value;

	/* Do the actual IO */
	uint16_t buf[32];
	const uint16_t num = mbbi ? dpvt->pterm->m_inputSize : 1;
	uint16_t addr = mbbi ? dpvt->pterm->m_inputStart - 1
						 : dpvt->pterm->m_inputStart +
							   (dpvt->channel - 2); // For non-mbbi records compute the coil offset.
													// channel is 1-based index, m_inputStart is also 1-based, but
													// modbus coils are 0-based, hence the -2
	assert(num <= sizeof(buf));
	status = dpvt->pterm->getEK9000IO(MODBUS_READ_DISCRETE_INPUTS, addr, buf, num);

	/* Error states */
	if (status) {
		recGblSetSevr(pRecord, READ_ALARM, INVALID_ALARM);
		/* Check type of err */
		if (status > 0x100) {
			LOG_WARNING(dpvt->pdrv, "EL10XX_read_record() for %s: %s\n", pRecord->name, devEK9000::ErrorToString(status));
			return 0;
		}
		LOG_WARNING(dpvt->pdrv, "EL10XX_read_record() for %s: %s\n", pRecord->name, devEK9000::ErrorToString(status));
		return 0;
	}

	// for mbbi, we need to composite our channel data into a single bit vector and leave .VAL alone
	if (mbbi) {
		uint32_t outVec = 0;
		STATIC_ASSERT(sizeof(buf) / sizeof(buf[0]) <= sizeof(outVec) * 8);
		for (size_t i = 0; i < ArraySize(buf); ++i)
			outVec |= (buf[i] << i) & (1 << i);
		// Template hack because we have no if constexpr before C++17
		set_mbbi_rval(pRecord, outVec);
	}
	else {
		pRecord->val = buf[0];
		pRecord->rval = buf[0];
	}
	pRecord->udf = FALSE;
	return 0;
}

struct devEL10XX_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
} devEL10XX = {
	5,
	(DEVSUPFUN)EL10XX_dev_report,
	(DEVSUPFUN)EL10XX_init,
	(DEVSUPFUN)EL10XX_init_record<biRecord>,
	(DEVSUPFUN)EL10XX_get_ioint_info,
	(DEVSUPFUN)EL10XX_read_record<biRecord>,
};

epicsExportAddress(dset, devEL10XX);

struct devEL10XXmbbi_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
} devEL10XX_mbbiDirect = {
	5,
	(DEVSUPFUN)EL10XX_dev_report,
	(DEVSUPFUN)EL10XX_init,
	(DEVSUPFUN)EL10XX_init_record<mbbiDirectRecord>,
	(DEVSUPFUN)EL10XX_get_ioint_info,
	(DEVSUPFUN)EL10XX_read_record<mbbiDirectRecord>,
};

epicsExportAddress(dset, devEL10XX_mbbiDirect);
