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
#include <vector>

#include "ekUtil.h"
#include "ekCoE.h"

#define PORT_PREFIX "PORT_"

#define STRUCT_SIZE_TO_MODBUS_SIZE(_x) ((_x % 2) == 0 ? (_x) / 2 : ((_x) / 2) + 1)

/* Beginning of the block of register space containing status info. Spans from 0x1010 <-> 0x1040 */
#define EK9000_STATUS_START 0x1010
#define EK9000_STATUS_END 0x1040

/* This device's error types */
enum {
	EK_EOK = 0,			/* OK */
	EK_EERR = 1,		/* Unspecified err */
	EK_EBADTERM = 2,	/* Bad terminal */
	EK_ENOCONN = 3,		/* Bad connection */
	EK_EBADPARAM = 4,	/* Bad parameter passed */
	EK_EBADPTR = 5,		/* Bad pointer */
	EK_ENODEV = 6,		/* Bad device */
	EK_ENOENT = 7,		/* No dir entry */
	EK_EWTCHDG = 8,		/* Watchdog error */
	EK_EBADTYP = 9,		/* Bad terminal type */
	EK_EBADIP = 10,		/* Bad IP format */
	EK_EBADPORT = 11,	/* Bad port # */
	EK_EADSERR = 12,	/* ADS error */
	EK_ETERMIDMIS = 13, /* Terminal id mismatch. e.g. term type is EL1124, but id is 2008 */
	EK_EBADMUTEX = 14,	/* Mutex error */
	EK_EMUTEXTIMEOUT = 15,
	EK_EBADTERMID = 16, /* Invalid terminal id */
	EK_EMODBUSERR = 17	/* Modbus error */
};

/* Buffered IO types */
enum EIOType {
	READ_ANALOG,  /* Analogous to MODBUS_READ_INPUT_REGISTER */
	READ_DIGITAL, /* Analogous to MODBUS_READ_DISCRETE_INPUTS */
	READ_STATUS	  /* For status registers (e.g. num TCP connections, hardware ver, etc) */
};

/* Forward decls */
class devEK9000;
class devEK9000Terminal;

std::list<devEK9000*>& GlobalDeviceList();

enum {
	TERMINAL_FAMILY_ANALOG = 0x1,
	TERMINAL_FAMILY_DIGITAL = 0x2
};

#define DevInfo(...)                                                                                                   \
	if (devEK9000::debugEnabled) {                                                                                     \
		epicsPrintf(__VA_ARGS__);                                                                                      \
	}
#define DevWarn(...)                                                                                                   \
	if (devEK9000::debugEnabled) {                                                                                     \
		epicsPrintf(__VA_ARGS__);                                                                                      \
	}
#define DevError(...)                                                                                                  \
	if (devEK9000::debugEnabled) {                                                                                     \
		epicsPrintf(__VA_ARGS__);                                                                                      \
	}

class devEK9000Terminal {
public:
	/* Copy constructor */
	devEK9000Terminal(devEK9000* device);

	void Init(uint32_t termid, int termindex);

	/* Process a record name. if outindex is nullptr, we are not expecting a channel selector at the end of the record
	 * name */
	static devEK9000Terminal* ProcessRecordName(const char* recname, int* outindex);

	static void GetTerminalInfo(int termid, int& inp_size, int& out_size);

	/* Do EK9000 IO */
	int doEK9000IO(int type, int startaddr, uint16_t* buf, int len);

	/* Same calling convention as above, but use the buffered data! */
	int getEK9000IO(EIOType type, int startaddr, uint16_t* buf, int len);

	void SetRecordName(const char* rec) {
		m_recordName = rec;
	}

public:
	/* Name of record */
	std::string m_recordName;
	/* Terminal family */
	int m_terminalFamily;
	/* Zero-based index of the terminal */
	int m_terminalIndex;
	/* the device */
	devEK9000* m_device;
	/* Terminal id, aka the 1124 in EL1124 */
	int m_terminalId;
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
class devEK9000 : public drvModbusAsyn {
private:
	friend class CDeviceMgr;
	friend class CTerminal;

	/* Mutex to lock modbus driver */
	epicsMutexId m_Mutex;

public:
	DELETE_CTOR(devEK9000());
	devEK9000(const char* portName, const char* octetPortName, int termCount, const char* ip);
	~devEK9000();

	/* List of attached terminals */
	/* TODO: Redefine terminal struct */
	std::vector<devEK9000Terminal*> m_terms;

	/* Number of attached terminals */
	int m_numTerms;

	std::string m_name;
	std::string m_octetPortName;
	std::string m_ip;

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
	IOSCANPVT m_status_io;

	int m_analog_status;
	int m_digital_status;
	int m_status_status;
	
	/* The actual analog/digital data */
	uint16_t* m_analog_buf;
	uint16_t* m_digital_buf;
	uint16_t m_analog_cnt;
	uint16_t m_digital_cnt;
	/* Buffer for status info */
	uint16_t m_status_buf[EK9000_STATUS_END - EK9000_STATUS_START + 1];

public:
	static devEK9000* FindDevice(const char* name);

public:
	/* Allows for better error handling (instead of using print statements to indicate error) */
	static devEK9000* Create(const char* name, const char* ip, int terminal_count);

	int AddTerminal(const char* name, uint32_t type, int position);

	/* Initializes a terminal (after it's been added). This should be called from the init_record routines */
	int InitTerminal(int termindex);

	/* Called to set proper image start addresses and such */
	bool ComputeTerminalMapping();

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

	/* Do a simple *blocking* I/O request. For optimized coupler IO use getEK9000IO */
	/* rw = 0 for read, rw = 1 for write */
	/* term = -1 for no terminal */
	/* len = number of regs to read */
	/* addr = starting addr */
	int doEK9000IO(int rw, uint16_t addr, uint16_t len, uint16_t* data);

	/**
	 * Get at the internal buffered IO on the coupler
	 * @param type IO type. Either READ_ANALOG, READ_DIGITAL or READ_STATUS. Write buffering not supported
	 * @param startaddr Address to start reading at
	 * @param buf Pointer to the buffer to receive the data
	 * @param len Number of registers to read. This is NOT a byte count!
	 */
	int getEK9000IO(EIOType type, int startaddr, uint16_t* buf, uint16_t len);

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

	devEK9000Terminal* TerminalByIndex(int idx) const {
		return m_terms[idx - 1];
	}

	asynUser* GetAsynUser() {
		return pasynUserSelf;
	}

public:
	/* Statics! */

	static bool debugEnabled;
	static int pollDelay;

public:
	/* Needed for the list impl */
	bool operator==(const devEK9000& other) const {
		return (strcmp(this->m_name.data(), other.m_name.data()) == 0);
	}
};

class DeviceLock FINAL {
	devEK9000& m_mutex;
	bool m_unlocked;
	int m_status;

public:
	DELETE_CTOR(DeviceLock());

	explicit DeviceLock(devEK9000* mutex) : m_mutex(*mutex), m_unlocked(false) {
		m_status = m_mutex.lock();
	}

	~DeviceLock() {
		if (!m_unlocked)
			m_mutex.unlock();
	}

	inline int status() const {
		return m_status;
	}
	inline bool valid() const {
		return m_status == asynSuccess;
	}

	inline void unlock() {
		if (!m_unlocked)
			m_mutex.unlock();
		m_unlocked = true;
	}
};
