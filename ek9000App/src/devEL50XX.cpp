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
#include "devEL50XX.h"

/* NOTE: There doesn't seem to be any EL5001/EL5002 in the Db directory for this! */

struct EL50XXDpvt_t {
	uint32_t tid;
	devEK9000Terminal* terminal;
	devEK9000* device;
	longinRecord* precord;
	union {
		EL5001Output_t el5001_output;
		EL5002Output_t el5002_output;
	};
};

static long el50xx_dev_report(int lvl);
static long el50xx_init(int after);
static long el50xx_init_record(void* precord);
static long el50xx_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt);
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

static long el50xx_dev_report(int) {
	return 0;
}

static long el50xx_init(int) {
	return 0;
}

static long el50xx_init_record(void* precord) {
	longinRecord* record = static_cast<longinRecord*>(precord);
	record->dpvt = calloc(1, sizeof(EL50XXDpvt_t));
	EL50XXDpvt_t* dpvt = static_cast<EL50XXDpvt_t*>(record->dpvt);

	/* Get the terminal */
	int channel = 0;
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(record->name, channel);
	if (!dpvt->terminal) {
		util::Error("EL50XX_init_record(): Unable to find terminal for record %s\n", record->name);
		return 1;
	}

	dpvt->device = dpvt->terminal->m_device;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->device->VerifyConnection()) {
		util::Error("EL50XX_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);
	dpvt->device->Unlock();
	dpvt->tid = termid;

	if (termid != dpvt->terminal->m_terminalId || termid == 0) {
		util::Error("EL50XX_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), record->name,
					termid);
		return 1;
	}

	return 0;
}

static long el50xx_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	EL50XXDpvt_t* dpvt = static_cast<EL50XXDpvt_t*>(pRecord->dpvt);

	if (!dpvt) {
		return 1;
	}

	*iopvt = dpvt->device->m_analog_io;
	return 0;
}

static long el50xx_read_record(void* prec) {
	longinRecord* precord = static_cast<longinRecord*>(prec);
	EL50XXDpvt_t* dpvt = static_cast<EL50XXDpvt_t*>(precord->dpvt);

	if (!dpvt || !dpvt->terminal || !dpvt->device)
		return 0;

	if (!dpvt->device->VerifyConnection()) {
		recGblSetSevr((longinRecord*)dpvt->precord, COMM_ALARM, INVALID_ALARM);
		return 0;
	}

	/* Read into a buffer that's plenty big enough for any terminal type */
	uint16_t data[32];
	dpvt->terminal->getEK9000IO(MODBUS_READ_INPUT_REGISTERS, dpvt->terminal->m_inputStart, data,
								STRUCT_SIZE_TO_MODBUS_SIZE(dpvt->terminal->m_inputSize));

	/* Handle individual terminal pdo types */
	switch (dpvt->tid) {
		case 5001:
			{
				EL5001Output_t* output = reinterpret_cast<EL5001Output_t*>(data);
				if (output->data.data_error || output->data.sync_err)
					recGblSetSevr(precord, READ_ALARM, INVALID_ALARM);
				if (output->data.frame_error)
					recGblSetSevr(precord, READ_ALARM, MAJOR_ALARM);
				precord->val = output->encoder_value;
				break;
			}
		case 5002:
			{
				EL5002Output_t* output = reinterpret_cast<EL5002Output_t*>(data);
				if (output->data_error)
					recGblSetSevr(precord, READ_ALARM, INVALID_ALARM);
				if (output->frame_error)
					recGblSetSevr(precord, COMM_ALARM, MAJOR_ALARM);
				precord->val = output->encoder_value;
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

static long el5042_dev_report(int lvl);
static long el5042_init_record(void* prec);
static long el5042_init(int after);
static long el5042_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt);
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
	(DEVSUPFUN)el5042_dev_report,
	(DEVSUPFUN)el5042_init,
	el5042_init_record,
	(DEVSUPFUN)el5042_get_ioint_info,
	el5042_read_record,
};

extern "C"
{
	epicsExportAddress(dset, devEL5042);
}

struct EL5042Dpvt_t {
	int channel;
	longinRecord* prec;
	devEK9000Terminal* terminal;
	devEK9000* device;
};

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

/*
-------------------------------------
Report on all EL5042 devices
-------------------------------------
*/
static long el5042_dev_report(int) {
	return 0;
}

/*
-------------------------------------
Initialize the specified record
-------------------------------------
*/
static long el5042_init_record(void* prec) {
	longinRecord* record = static_cast<longinRecord*>(prec);
	record->dpvt = calloc(1, sizeof(EL50XXDpvt_t));
	EL5042Dpvt_t* dpvt = static_cast<EL5042Dpvt_t*>(record->dpvt);

	/* Get the terminal */
	int channel = 0;
	dpvt->terminal = devEK9000Terminal::ProcessRecordName(record->name, channel);
	dpvt->channel = channel;
	if (!dpvt->terminal) {
		util::Error("EL5042_init_record(): Unable to find terminal for record %s\n", record->name);
		return 1;
	}

	dpvt->device = dpvt->terminal->m_device;
	dpvt->device->Lock();

	/* Check connection to terminal */
	if (!dpvt->device->VerifyConnection()) {
		util::Error("EL5042_init_record(): %s\n", devEK9000::ErrorToString(EK_ENOCONN));
		dpvt->device->Unlock();
		return 1;
	}

	/* Check that slave # is OK */
	uint16_t termid = 0;
	dpvt->terminal->m_device->ReadTerminalID(dpvt->terminal->m_terminalIndex, termid);
	dpvt->device->Unlock();
	dpvt->prec = static_cast<longinRecord*>(prec);
	if (termid != dpvt->terminal->m_terminalId || termid == 0) {
		util::Error("EL5042_init_record(): %s: %s != %u\n", devEK9000::ErrorToString(EK_ETERMIDMIS), record->name,
					termid);
		return 1;
	}
	return 0;
}

/*
-------------------------------------
Initialize the device support module
-------------------------------------
*/
static long el5042_init(int) {
	return 0;
}

/*
---------------------------------------
Called to update the I/O interrupt data
---------------------------------------
*/
static long el5042_get_ioint_info(int cmd, void* prec, IOSCANPVT* iopvt) {
	UNUSED(cmd);
	struct dbCommon* pRecord = static_cast<struct dbCommon*>(prec);
	EL5042Dpvt_t* dpvt = static_cast<EL5042Dpvt_t*>(pRecord->dpvt);

	if (!dpvt) {
		return 1;
	}

	*iopvt = dpvt->device->m_analog_io;
	return 0;
}

/*
-------------------------------------
Called to read the specified record
-------------------------------------
*/
static long el5042_read_record(void* prec) {
	longinRecord* precord = static_cast<longinRecord*>(prec);
	EL5042Dpvt_t* dpvt;
	EL5042InputPDO_t* pdo;

	dpvt = static_cast<EL5042Dpvt_t*>(precord->dpvt);
	if (!dpvt) {
		return 0;
	}

	/* Read the stuff */
	uint16_t buf[32];
	uint16_t loc = dpvt->terminal->m_inputStart + ((dpvt->channel - 1) * 3);
	dpvt->terminal->getEK9000IO(MODBUS_READ_INPUT_REGISTERS, loc, buf,
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
