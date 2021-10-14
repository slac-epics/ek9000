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
// Name: devEK9000.h
// Purpose: Device support for EK9000 and it's associated
// devices
// Authors: Jeremy L.
// Date Created: July 6, 2019
//======================================================//
#pragma once

/* Record types */
#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>

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
#include <boRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <epicsStdio.h>
#include <errlog.h>
#include <epicsMessageQueue.h>

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <functional>
#include <list>

#include "ekUtil.h"
#include "ekDiag.h"
#include "ekCoE.h"

#define PORT_PREFIX "PORT_"

#define STRUCT_SIZE_TO_MODBUS_SIZE(_x) ((_x % 2) == 0 ? (_x) / 2 : ((_x) / 2) + 1)

/* This device's error types */
#define EK_EOK 0		 /* OK */
#define EK_EERR 1		 /* Unspecified err */
#define EK_EBADTERM 2	 /* Bad terminal */
#define EK_ENOCONN 3	 /* Bad connection */
#define EK_EBADPARAM 4	 /* Bad parameter passed */
#define EK_EBADPTR 5	 /* Bad pointer */
#define EK_ENODEV 6		 /* Bad device */
#define EK_ENOENT 7		 /* No dir entry */
#define EK_EWTCHDG 8	 /* Watchdog error */
#define EK_EBADTYP 9	 /* Bad terminal type */
#define EK_EBADIP 10	 /* Bad IP format */
#define EK_EBADPORT 11	 /* Bad port # */
#define EK_EADSERR 12	 /* ADS error */
#define EK_ETERMIDMIS 13 /* Terminal id mismatch. e.g. term type is EL1124, but id is 2008 */
#define EK_EBADMUTEX 14	 /* Mutex error */
#define EK_EMUTEXTIMEOUT 15
#define EK_EBADTERMID 16 /* Invalid terminal id */
#define EK_EMODBUSERR 17 /* Modbus error */

/* Forward decls */
class devEK9000;
class devEK9000Terminal;

extern std::list<devEK9000*> g_Devices;
extern bool g_bDebug;

std::list<devEK9000*>& GlobalDeviceList();

#define TERMINAL_FAMILY_ANALOG 0x1
#define TERMINAL_FAMILY_DIGITAL 0x2

#define DevInfo(fmt, ...)                                                                                              \
	if (g_bDebug) {                                                                                                    \
		util::Log(fmt, __VA_ARGS__);                                                                                   \
	}
#define DevWarn(fmt, ...)                                                                                              \
	if (g_bDebug) {                                                                                                    \
		util::Warn(fmt, __VA_ARGS__);                                                                                  \
	}
#define DevError(fmt, ...)                                                                                             \
	if (g_bDebug) {                                                                                                    \
		util::Error(fmt, __VA_ARGS__);                                                                                 \
	}

struct LinkParameter_t {
	char* key;
	char* value;
};

struct LinkSpecification_t {
	LinkParameter_t* params;
	int numParams;
};

typedef struct {
	class devEK9000* pdrv;
	int slave, terminal, channel;
	int baseaddr, len;
	LinkSpecification_t linkSpec;
	char* terminalType;
	char* mapping;
	char* representation;
} TerminalDpvt_t;

enum ELinkType {
	BAD = 0,
	LINK_INST_IO,
};

class devEK9000Terminal {
public:
	/* Copy constructor */
	devEK9000Terminal(const devEK9000Terminal& other);
	devEK9000Terminal();
	~devEK9000Terminal();

	/* device is parent device, termid is 3064 in el3064 */
	static devEK9000Terminal* Create(devEK9000* device, uint32_t termid, int termindex, const char* record);

	/* Process record name */
	static devEK9000Terminal* ProcessRecordName(const char* recname, int& outindex, char* outname);

	static void GetTerminalInfo(int termid, int& inp_size, int& out_size);

	/* Do EK9000 IO */
	int doEK9000IO(int type, int startaddr, uint16_t* buf, size_t len);

	/* Same calling convention as above, but use the buffered data! */
	int getEK9000IO(int type, int startaddr, uint16_t* buf, size_t len);

	template <class T> bool CoEWriteParameter(coe::param_t param, T value);

	template <class T> bool CoEReadParameter(coe::param_t param, T& outval);

public:
	/* Name of record */
	char* m_recordName;
	/* Terminal family */
	int m_terminalFamily;
	/* Zero-based index of the terminal */
	int m_terminalIndex;
	/* the device */
	devEK9000* m_device;
	/* Terminal id, aka the 1124 in EL1124 */
	int m_terminalId;
	/* Number of inputs */
	int m_inputs;
	/* Number of outputs */
	int m_outputs;
	/* Size of inputs */
	int m_inputSize;
	/* Size of outputs */
	int m_outputSize;
	/* input image start */
	int m_inputStart;
	/* Output image start */
	int m_outputStart;
};

//==========================================================//
// class devEK9000
//		Holds useful vars for interacting with EK9000/EL****
//		hardware
//==========================================================//

/* This holds various useful info about each ek9000 coupler */
class devEK9000 {
private:
	friend class CDeviceMgr;
	friend class CTerminal;

	/* Mutex to lock modbus driver */
	epicsMutexId m_Mutex;

public:
	/* Device info */
	// EK9000Device m_pDevice;

	/* List of attached terminals */
	/* TODO: Redefine terminal struct */
	devEK9000Terminal* m_terms;

	/* Number of attached terminals */
	int m_numTerms;

	/* The driver */
	drvModbusAsyn* m_driver;

	char* m_name;
	char* m_portName;
	char* m_ip;

	bool m_connected;
	bool m_init;

	/* Enable/disable debugging messages */
	bool m_debug;

	/* last device err */
	int m_error;

	int LastADSErr;

	/* Interrupts for analog/digital inputs */
	IOSCANPVT m_analog_io;
	IOSCANPVT m_digital_io;
	/* The actual analog/digital data */
	uint16_t  *m_analog_buf;
	uint16_t  *m_digital_buf;
	uint16_t  m_analog_cnt;
	uint16_t  m_digital_cnt;

public:
	devEK9000();

	/* Make  sure to free everything */
	~devEK9000();

public:
	static devEK9000* FindDevice(const char* name);

public:
	/* Allows for better error handling (instead of using print statements to indicate error) */
	static devEK9000* Create(const char* name, const char* ip, int terminal_count);

	int AddTerminal(const char* name, int type, int position);

	devEK9000Terminal* GetTerminal(const char* recordname);

	devEK9000Terminal* GetAllTerminals() {
		return m_terms;
	}

	/* Initializes a terminal (after it's been added). This should be called from the init_record routines */
	int InitTerminal(int termindex);

	/* Called to set proper image start addresses and such */
	int InitTerminals();

	/* Grab mutex */
	int Lock();
	/* Unlock it */
	void Unlock();

public:
	/* Error handling functions */

	/* Returns the last error & pops it */
	int LastError();

	/* Returns a string representing last error */
	const char* LastErrorString();

	/* error to string */
	static const char* ErrorToString(int err);

public:
	/* Utils for reading/writing */

	/* Verify connection using asynUser */
	/* Return 1 for connect */
	int VerifyConnection() const;

	/* Do a simple I/O thing */
	/* rw = 0 for read, rw = 1 for write */
	/* term = -1 for no terminal */
	/* len = number of regs to read */
	/* addr = starting addr */
	int doEK9000IO(int rw, uint16_t addr, uint16_t len, uint16_t* data);

	/* Do CoE I/O */
	int doCoEIO(int rw, uint16_t term, uint16_t index, uint16_t len, uint16_t* data, uint16_t subindex,
				uint16_t reallen = 0);

	/* Reads the value of the CoE register */
	int ReadCOECode();

	/* Reads coupler type */
	int ReadTerminalType(uint16_t termid, int& id);

	/* Reads coupler id */
	int ReadCouplerID(char* outbuf, size_t& outbufsize);

	/* Reads the length of the process image */
	int ReadProcessImageSize(uint16_t& anal_out, uint16_t& anal_in, uint16_t& dig_out, uint16_t& dig_in);

	/* Reads the current watchdog time */
	int ReadWatchdogTime(uint16_t& out);

	/* Read the number of fallbacks triggered */
	int ReadNumFallbacksTriggered(uint16_t& out);

	/* Read the number of tcp connections */
	int ReadNumTCPConnections(uint16_t& out);

	/* Read hardware/software versions */
	int ReadVersionInfo(uint16_t& hardver, uint16_t& softver_major, uint16_t& softver_minor, uint16_t& softver_patch);

	/* Read the serial number */
	int ReadSerialNumber(uint16_t& sn);

	/* Read manfacturing date */
	int ReadMfgDate(uint16_t& day, uint16_t& mon, uint16_t& year);

	/* Read EBus status */
	int ReadEBusStatus(uint16_t& stauts);

	/* Write the watcdog time */
	int WriteWatchdogTime(uint16_t time);

	/* Reset watchdog timer */
	int WriteWatchdogReset();

	/* Write watchdog type */
	int WriteWatchdogType(uint16_t type);

	/* Write fallback mode */
	int WriteFallbackMode(uint16_t mode);

	/* Enable/disable writing to second modbus client */
	int WriteWritelockMode(uint16_t mode);

	/* Read terminal type */
	int ReadTerminalID(uint16_t termid, uint16_t& termidout);

	/* Poll the ek9000 until data is ready/error */
	/* Return 0 for OK, 1 for error */
	/* Duration is the space between each request, timeout is the number of requests to timeout after */
	inline int Poll(float duration, int timeout);

	/* Try connect to terminal with CoE */
	/* Returns 1 for connection, 0 for not */
	int CoEVerifyConnection(uint16_t termid);

public:
	/* Error reporting function, only prints on debug */
	// void ReportError(int errorcode, const char* _msg = NULL);

	/* Enable/disable debug */
	// void EnableDebug(bool enabled) { m_debug = enabled; };

public:
	/* Needed for the list impl */
	bool operator==(const devEK9000& other) const {
		return (strcmp(this->m_name, other.m_name) == 0);
	}

	/* Couple utility functions */
	template <class RecordT> static bool setupCommonDpvt(RecordT* prec, TerminalDpvt_t& dpvt);

	template <class RecordT> static void destroyDpvt(RecordT* prec, TerminalDpvt_t& dpvt);

	static TerminalDpvt_t emptyDpvt() {
		TerminalDpvt_t dpvt;
		memset(&dpvt, 0, sizeof(TerminalDpvt_t));
		return dpvt;
	}

	static void DestroyLinkSpecification(LinkSpecification_t& spec);
	static bool ParseLinkSpecification(const char* link, ELinkType linkType, LinkSpecification_t& outSpec);
};

/**
 * We also handle some backwards compatibility here.
 */
template <class RecordT> bool devEK9000::setupCommonDpvt(RecordT* prec, TerminalDpvt_t& dpvt) {
	const char* function = "util::setupCommonDpvt<RecordT>()";
	std::list<devEK9000*>& devList = GlobalDeviceList();

	if (!devEK9000::ParseLinkSpecification(prec->inp.text, LINK_INST_IO, dpvt.linkSpec)) {
		/* Try to work with legacy stuff */
		// TODO: STUB
		return false;
	}

	/* Parse the params passed via INST_IO stuff */
	for (int i = 0; i < dpvt.linkSpec.numParams; i++) {
		LinkParameter_t param = dpvt.linkSpec.params[i];

		/* Device name */
		if (strcmp(param.key, "device") == 0) {
			bool found = false;
			std::list<devEK9000*>& devList = GlobalDeviceList();
			for (std::list<devEK9000*>::iterator x = devList.begin(); x != devList.end(); ++x) {
				// for (const auto& x : GlobalDeviceList()) {
				if (strcmp((*x)->m_name, param.value) == 0) {
					dpvt.pdrv = *x;
					found = true;
					break;
				}
			}
			if (!found) {
				epicsPrintf("%s (when parsing %s): invalid device name: %s\n", function, prec->name, param.value);
				return false;
			}
		}
		/* Terminal position in rail (1 based) */
		else if (strcmp(param.key, "terminal") == 0) {
			int term = atoi(param.value);
			/* Max supported devices by the EK9K is 255 */
			if (term < 0 || term > 255) {
				epicsPrintf("%s (when parsing %s): invalid slave number: %i\n", function, prec->name, term);
				return false;
			}
			dpvt.slave = term;
		}
		/* Channel number */
		else if (strcmp(param.key, "channel") == 0) {
			int channel = atoi(param.value);
			/* No real max here, but I think it's good to limit this to 8k as nothing has this many channels */
			if (channel < 0 || channel > 8192) {
				epicsPrintf("%s (when parsing %s): invalid channel: %i\n", function, prec->name, channel);
				return false;
			}
			dpvt.channel = channel;
		}
		/* Terminal type string e.g. EL3064 */
		else if (strcmp(param.key, "type") == 0) {
			dpvt.terminalType = epicsStrDup(param.value);
		}
		/* Representation. Generally used for analog termianls */
		else if (strcmp(param.key, "repres") == 0) {
			dpvt.representation = epicsStrDup(param.value);
		}
		/* Mapping type. Terminals generally have different mapping types. */
		/* Ex: compact, compact w/status, standard, standard w/status */
		/* Can be changed by iocsh */
		else if (strcmp(param.key, "mapping") == 0) {
			dpvt.mapping = epicsStrDup(param.value);
		}
		else {
			epicsPrintf("%s (when parsing %s): ignored unknown param %s\n", function, prec->name, param.key);
		}
	}

	if (!dpvt.mapping)
		dpvt.mapping = epicsStrDup("");
	if (!dpvt.terminalType)
		dpvt.terminalType = epicsStrDup("");
	if (!dpvt.representation)
		dpvt.representation = epicsStrDup("");
	return true;
}
