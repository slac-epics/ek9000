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

#define STRUCT_SIZE_TO_MODBUS_SIZE(_x) ((_x % 2) == 0 ? (_x) / 2 : ((_x)/2) + 1)

/* This device's error types */
#define EK_EOK 0		 /* OK */
#define EK_EERR 1		 /* Unspecified err */
#define EK_EBADTERM 2	/* Bad terminal */
#define EK_ENOCONN 3	 /* Bad connection */
#define EK_EBADPARAM 4   /* Bad parameter passed */
#define EK_EBADPTR 5	 /* Bad pointer */
#define EK_ENODEV 6		 /* Bad device */
#define EK_ENOENT 7		 /* No dir entry */
#define EK_EWTCHDG 8	 /* Watchdog error */
#define EK_EBADTYP 9	 /* Bad terminal type */
#define EK_EBADIP 10	 /* Bad IP format */
#define EK_EBADPORT 11   /* Bad port # */
#define EK_EADSERR 12	/* ADS error */
#define EK_ETERMIDMIS 13 /* Terminal id mismatch. e.g. term type is EL1124, but id is 2008 */
#define EK_EBADMUTEX 14	/* Mutex error */
#define EK_EMUTEXTIMEOUT 15
#define EK_EBADTERMID 16 /* Invalid terminal id */
#define EK_EMODBUSERR 17 /* Modbus error */

/* Forward decls */
class CEK9000Device;
struct CTerminal;

extern std::list<CEK9000Device*> g_Devices;
extern bool g_bDebug;

std::list<CEK9000Device*>& GlobalDeviceList();

#define TERMINAL_FAMILY_ANALOG 0x1
#define TERMINAL_FAMILY_DIGITAL 0x2

void Info(const char* fmt, ...);
void Warning(const char* fmt, ...);
void Error(const char* fmt, ...);

#define DevInfo(fmt, ...) if(g_bDebug) { Info(fmt, __VA_ARGS__); }
#define DevWarn(fmt, ...) if(g_bDebug) { Warning(fmt, __VA_ARGS__); }
#define DevError(fmt, ...) if(g_bDebug) { Error(fmt, __VA_ARGS__); }

struct LinkParameter_t 
{
	char* key;
	char* value;
};

struct LinkSpecification_t
{
	LinkParameter_t* params;
	int numParams;
};

typedef struct 
{
	class CEK9000Device* pdrv;
	int slave, terminal, channel;
	int baseaddr, len;
	LinkSpecification_t linkSpec;
	char* terminalType;
	char* mapping;
	char* representation;
} terminal_dpvt_t;

enum ELinkType 
{
	BAD = 0,
	LINK_INST_IO,
};



class CTerminal
{
public:
	/* Copy constructor */
	CTerminal(const CTerminal& other);
	CTerminal();
	~CTerminal();

	/* device is parent device, termid is 3064 in el3064 */
	static CTerminal* Create(CEK9000Device* device, uint32_t termid, int termindex, const char* record);

	/* Process record name */
	static CTerminal* ProcessRecordName(const char* recname, int& outindex, char* outname);

	static void GetTerminalInfo(int termid, int& inp_size, int& out_size);

	/* Do EK9000 IO */
	int doEK9000IO(int type, int startaddr, uint16_t* buf, size_t len);

	template<class T>
	bool CoEWriteParameter(coe::param_t param, T value);

	template<class T>
	bool CoEReadParameter(coe::param_t param, T& outval);

public:
	/* Name of record */
	char *m_pRecordName = NULL;
	/* Terminal family */
	int m_TerminalFamily = 0;
	/* Zero-based index of the terminal */
	int m_nTerminalIndex = 0;
	/* the device */
	CEK9000Device *m_pDevice = NULL;
	/* Terminal id, aka the 1124 in EL1124 */
	int m_nTerminalID = 0;
	/* Number of inputs */
	int m_nInputs = 0;
	/* Number of outputs */
	int m_nOutputs = 0;
	/* Size of inputs */
	int m_nInputSize = 0;
	/* Size of outputs */
	int m_nOutputSize = 0;
	/* input image start */
	int m_nInputStart = 0;
	/* Output image start */
	int m_nOutputStart = 0;
};

//==========================================================//
// class CEK9000Device
//		Holds useful vars for interacting with EK9000/EL****
//		hardware
//==========================================================//

/* This holds various useful info about each ek9000 coupler */
class CEK9000Device
{
private:
	friend class CDeviceMgr;
	friend class CTerminal;

	/* Mutex to lock modbus driver */
	epicsMutexId m_Mutex;

public:
	/* Device info */
	//EK9000Device m_pDevice;

	/* List of attached terminals */
	/* TODO: Redefine terminal struct */
	CTerminal *m_pTerms = NULL;

	/* Number of attached terminals */
	int m_nTerms = 0;

	/* The driver */
	drvModbusAsyn *m_pDriver = NULL;

	char *m_pName = NULL;
	char *m_pPortName = NULL;
	char *m_pIP = NULL;

	bool m_bConnected = false;
	bool m_bInit = false;

	/* Enable/disable debugging messages */
	bool m_bDebug = false;

	/* last device err */
	int m_nError = EK_EOK;

	/* Message queue */
	epicsMessageQueue* queue;

	int LastADSErr = 0;

public:
	CEK9000Device();

	/* Make  sure to free everything */
	~CEK9000Device();

public:
	static CEK9000Device* FindDevice(const char* name);

public:
	void QueueCallback(void(*callback)(void*), void* rec);

	void ExecuteCallbacks();

	/* Allows for better error handling (instead of using print statements to indicate error) */
	static CEK9000Device *Create(const char *name, const char *ip, int terminal_count);

	int AddTerminal(const char *name, int type, int position);

	CTerminal *GetTerminal(const char *recordname);

	CTerminal *GetAllTerminals() { return m_pTerms; }

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
	const char *LastErrorString();

	/* error to string */
	static const char *ErrorToString(int err);
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
	int doEK9000IO(int rw, uint16_t addr, uint16_t len, uint16_t *data);

	/* Do CoE I/O */
	int doCoEIO(int rw, uint16_t term, uint16_t index, uint16_t len, uint16_t *data, uint16_t subindex, uint16_t reallen = 0);

	/* same as doEK9000IO except this only operates on the ek9000 itself, term is not used */
	int doCouplerIO(int rw, uint16_t term, uint16_t len, uint16_t addr, uint16_t *data, uint16_t subindex = 0);

	/* Reads the value of the CoE register */
	int ReadCOECode();

	/* Reads coupler type */
	int ReadTerminalType(uint16_t termid, int &id);

	/* Reads coupler id */
	int ReadCouplerID(char *outbuf, size_t &outbufsize);

	/* Reads the length of the process image */
	int ReadProcessImageSize(uint16_t &anal_out, uint16_t &anal_in, uint16_t &dig_out, uint16_t &dig_in);

	/* Reads the current watchdog time */
	int ReadWatchdogTime(uint16_t &out);

	/* Read the number of fallbacks triggered */
	int ReadNumFallbacksTriggered(uint16_t &out);

	/* Read the number of tcp connections */
	int ReadNumTCPConnections(uint16_t &out);

	/* Read hardware/software versions */
	int ReadVersionInfo(uint16_t &hardver, uint16_t &softver_major, uint16_t &softver_minor, uint16_t &softver_patch);

	/* Read the serial number */
	int ReadSerialNumber(uint16_t &sn);

	/* Read manfacturing date */
	int ReadMfgDate(uint16_t &day, uint16_t &mon, uint16_t &year);

	/* Read EBus status */
	int ReadEBusStatus(uint16_t &stauts);

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
	int ReadTerminalID(uint16_t termid, uint16_t &termidout);

	/* Poll the ek9000 until data is ready/error */
	/* Return 0 for OK, 1 for error */
	/* Duration is the space between each request, timeout is the number of requests to timeout after */
	inline int Poll(float duration, int timeout);

	/* Try connect to terminal with CoE */
	/* Returns 1 for connection, 0 for not */
	int CoEVerifyConnection(uint16_t termid);
public:
	/* Error reporting function, only prints on debug */
	//void ReportError(int errorcode, const char* _msg = NULL);

	/* Enable/disable debug */
	//void EnableDebug(bool enabled) { m_bDebug = enabled; };

public:
	/* Needed for the list impl */
	bool operator==(const CEK9000Device &other) const
	{
		return (strcmp(this->m_pName, other.m_pName) == 0);
	}


	/* Couple utility functions */
	template<class RecordT>
	static bool setupCommonDpvt(RecordT* prec, terminal_dpvt_t& dpvt);

	template<class RecordT>
	static void destroyDpvt(RecordT* prec, terminal_dpvt_t& dpvt);
	
	static terminal_dpvt_t emptyDpvt()
	{
		terminal_dpvt_t dpvt;
		memset(&dpvt, 0, sizeof(terminal_dpvt_t));
		return dpvt;
	}

	static void DestroyLinkSpecification(LinkSpecification_t& spec);
	static bool ParseLinkSpecification(const char* link, ELinkType linkType, LinkSpecification_t& outSpec);
};

/**
 * We also handle some backwards compatibility here.
 */ 
template<class RecordT>
bool CEK9000Device::setupCommonDpvt(RecordT* prec, terminal_dpvt_t& dpvt)
{
	const char* function = "util::setupCommonDpvt<RecordT>()";
	if(!CEK9000Device::ParseLinkSpecification(prec->inp.text, LINK_INST_IO, dpvt.linkSpec))
	{
		/* Try to work with legacy stuff */
		// TODO: STUB
		return false;
	}

	/* Parse the params passed via INST_IO stuff */
	for(int i = 0; i < dpvt.linkSpec.numParams; i++)
	{
		LinkParameter_t param = dpvt.linkSpec.params[i];
		
		/* Device name */
		if(strcmp(param.key, "device") == 0)
		{
			bool found = false;
			for(const auto& x : GlobalDeviceList())
			{
				if(strcmp(x->m_pName, param.value) == 0) {
					dpvt.pdrv = x;
					found = true;
					break;
				}
			}
			if(!found) {
				epicsPrintf("%s (when parsing %s): invalid device name: %s\n", function,
					prec->name, param.value);
				return false;
			}
		}
		/* Terminal position in rail (1 based) */
		else if(strcmp(param.key, "terminal") == 0)
		{
			int term = atoi(param.value);
			/* Max supported devices by the EK9K is 255 */
			if(term < 0 || term > 255) {
				epicsPrintf("%s (when parsing %s): invalid slave number: %i\n", function, 
					prec->name, term);
				return false;
			}
			dpvt.slave = term;
		}
		/* Channel number */
		else if(strcmp(param.key, "channel") == 0)
		{
			int channel = atoi(param.value);
			/* No real max here, but I think it's good to limit this to 8k as nothing has this many channels */
			if(channel < 0 || channel > 8192) {
				epicsPrintf("%s (when parsing %s): invalid channel: %i\n", function, 
					prec->name, channel);
				return false;
			}
			dpvt.channel = channel;
		}
		/* Terminal type string e.g. EL3064 */
		else if(strcmp(param.key, "type") == 0)
		{
			dpvt.terminalType = epicsStrDup(param.value);
		}
		/* Representation. Generally used for analog termianls */
		else if(strcmp(param.key, "repres") == 0)
		{
			dpvt.representation = epicsStrDup(param.value);
		}
		/* Mapping type. Terminals generally have different mapping types. */
		/* Ex: compact, compact w/status, standard, standard w/status */
		/* Can be changed by iocsh */
		else if(strcmp(param.key, "mapping") == 0)
		{
			dpvt.mapping = epicsStrDup(param.value);
		}
		else
		{
			epicsPrintf("%s (when parsing %s): ignored unknown param %s\n", function, prec->name, param.key);
		}
	}
}


template<class RecordT>
void CEK9000Device::destroyDpvt(RecordT* prec, terminal_dpvt_t& dpvt)
{

}


