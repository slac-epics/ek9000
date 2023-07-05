/*
 *
 * EL50XX device support (encoder interfaces)
 *
 */
#include <epicsPrint.h>
#include <dbScan.h>
#include <alarm.h>
#include <recGbl.h>

#include <longinRecord.h>

#include "devEK9000.h"

#include "terminal_types.g.h"

// PDO definitions
#pragma pack(1)
struct EL5001Status_t {
	uint8_t data_error : 1;
	uint8_t frame_error : 1;
	uint8_t power_fail : 1;
	uint8_t data_mismatch : 1;
	uint8_t _r1 : 1;
	uint8_t sync_err : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
};

STATIC_ASSERT(sizeof(EL5001Status_t) == sizeof(uint8_t));

/* Input data from an el5001 terminal. el5001 from fw11 onwards also supports the EL5002Input_t PDO type  */
struct EL5001Input_t {
	union {
		uint8_t status_byte;
		EL5001Status_t data;
	};
	uint32_t encoder_value;
	// This is additional padding to keep EL5001Input_t a multiple of 2. The ek9000 rounds up when mapping terminals to
	// input/holding register space
	// meaning, if a PDO is 3 bytes, it will be mapped as if it were 4 bytes (2 registers), 5 bytes as if it were 6 and
	// so on.
	uint8_t _pad;
};

/* Input data from an el5002 slave, given it has the extended status byte enabled */
struct EL5002Input_t {
	uint8_t data_error : 1;
	uint8_t frame_error : 1;
	uint8_t power_fail : 1;
	uint8_t _r1 : 5;
	uint8_t _r2;
	uint32_t encoder_value;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL5001Input_t, EL5001);
DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL5002Input_t, EL5002);

/*
-------------------------------------
Common routines for EL50XX terminals
-------------------------------------
*/

static long el50xx_dev_report(int) {
	return 0;
}

static long el50xx_init(int) {
	return 0;
}

static long el50xx_init_record(void* precord) {
	longinRecord* record = static_cast<longinRecord*>(precord);
	record->dpvt = util::allocDpvt();
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(record->dpvt);

	/* Get the terminal */
	if (!util::setupCommonDpvt(record, *dpvt)) {
		LOG_ERROR(dpvt->pdrv, "Unable to setup dpvt for %s\n", record->name);
		return 1;
	}

	DeviceLock lock(dpvt->pdrv);

	if (!lock.valid()) {
		LOG_ERROR(dpvt->pdrv, "unable to obtain device lock\n");
		return 1;
	}

	/* Check connection to terminal */
	if (!dpvt->pdrv->VerifyConnection()) {
		LOG_ERROR(dpvt->pdrv, "%s\n", devEK9000::ErrorToString(EK_ENOCONN));
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->pterm->m_device->ReadTerminalID(dpvt->pterm->m_terminalIndex, termid);
	lock.unlock();

	if (termid != dpvt->pterm->m_terminalId || termid == 0) {
		LOG_ERROR(dpvt->pdrv, "%s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), record->name, termid);
		return 1;
	}

	return 0;
}

static long el50xx_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(pRecord->dpvt);
	if (!util::DpvtValid(dpvt))
		return 1;

	*iopvt = dpvt->pdrv->m_analog_io;
	return 0;
}

static long el50xx_read_record(void* precord);

struct devEL50XX_t {
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
	(DEVSUPFUN)el50xx_get_ioint_info,
	el50xx_read_record,
};

extern "C"
{
	epicsExportAddress(dset, devEL50XX);
}

static long el50xx_read_record(void* prec) {
	longinRecord* precord = static_cast<longinRecord*>(prec);
	TerminalDpvt_t* dpvt = static_cast<TerminalDpvt_t*>(precord->dpvt);

	if (!dpvt || !dpvt->pterm || !dpvt->pdrv)
		return 0;

	if (!dpvt->pdrv->VerifyConnection()) {
		recGblSetSevr(precord, COMM_ALARM, INVALID_ALARM);
		return 0;
	}

	/* Read into a buffer that's plenty big enough for any terminal type */
	union {
		EL5001Input_t el5001;
		EL5002Input_t el5002;
	} data;
	dpvt->pterm->getEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->pterm->m_inputStart, reinterpret_cast<uint16_t*>(&data),
							 dpvt->pterm->m_inputSize);

	/* Handle individual terminal pdo types */
	switch (dpvt->terminalType) {
		case 5001:
			{
				EL5001Input_t& input = data.el5001;
				if (input.data.data_error || input.data.sync_err)
					recGblSetSevr(precord, READ_ALARM, INVALID_ALARM);
				if (input.data.frame_error)
					recGblSetSevr(precord, READ_ALARM, MAJOR_ALARM);
				precord->val = input.encoder_value;
				break;
			}
		case 5002:
			{
				EL5002Input_t& input = data.el5002;
				if (input.data_error)
					recGblSetSevr(precord, READ_ALARM, INVALID_ALARM);
				if (input.frame_error)
					recGblSetSevr(precord, COMM_ALARM, MAJOR_ALARM);
				precord->val = input.encoder_value;
				break;
			}
		default:
			{
				/* Raise invalid alarm if we don't have a tid */
				recGblSetSevr(precord, READ_ALARM, INVALID_ALARM);
			}
	}
	precord->udf = FALSE;
	return 0;
}

static long el5042_read_record(void* prec);

struct devEL5042_t {
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN read_record;
} devEL5042 = {
	5,
	(DEVSUPFUN)el50xx_dev_report,
	(DEVSUPFUN)el50xx_init,
	el50xx_init_record,
	(DEVSUPFUN)el50xx_get_ioint_info,
	el5042_read_record,
};

extern "C"
{
	epicsExportAddress(dset, devEL5042);
}

#pragma pack(1)
struct EL5042InputPDO_t {
	uint8_t warning : 1;
	uint8_t error : 1;
	uint8_t ready : 1;
	uint8_t _r1 : 5;
	uint8_t _r2 : 4;
	uint8_t diag : 1;
	uint8_t txpdo_state : 1;
	uint8_t input_cycle_counter : 2;
	uint32_t position;
};
#pragma pack()

DEFINE_SINGLE_CHANNEL_INPUT_PDO(EL5042InputPDO_t, EL5042);

/*
-------------------------------------
Called to read the specified record
-------------------------------------
*/
static long el5042_read_record(void* prec) {
	longinRecord* precord = static_cast<longinRecord*>(prec);
	TerminalDpvt_t* dpvt;
	EL5042InputPDO_t* pdo;

	dpvt = static_cast<TerminalDpvt_t*>(precord->dpvt);
	if (!dpvt) {
		return 0;
	}

	/* Read the stuff */
	uint16_t buf[32];
	uint16_t loc = dpvt->pterm->m_inputStart + ((dpvt->channel - 1) * 3);
	dpvt->pterm->getEK9000IO(MODBUS_READ_INPUT_REGISTERS, loc, buf,
							 STRUCT_SIZE_TO_MODBUS_SIZE(sizeof(EL5042InputPDO_t)));

	/* Cast it to our pdo type */
	pdo = reinterpret_cast<EL5042InputPDO_t*>(buf);

	/* Update our params */
	precord->val = pdo->position;

	/* Check for any read alarms */
	if (pdo->warning) {
		recGblSetSevr(precord, READ_ALARM, MINOR_ALARM);
	}

	/* Check for any errors */
	if (pdo->error) {
		recGblSetSevr(precord, READ_ALARM, MAJOR_ALARM);
	}

	precord->udf = FALSE;
	return 0;
}
