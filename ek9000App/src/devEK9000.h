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
#ifndef _DEV_EK9K_H_
#define _DEV_EK9K_H_

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

#define PORT_PREFIX "PORT_"

/* Choice strings */
#define EL2008_STRING "EL2008"
#define EL1124_STRING "EL1124"
#define EL3064_STRING "EL3064"
#define EL1XXX_STRING "EL1XXX"
#define EL2XXX_STRING "EL2XXX"
#define EL3XXX_STRING "EL3XXX"
#define EL4XXX_STRING "EL4XXX"
#define EL5XXX_STRING "EL5XXX"
#define EL6XXX_STRING "EL6XXX"
#define EL7XXX_STRING "EL7XXX"
#define EL9XXX_STIRNG "EL9XXX"

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

#define IMAGE_TYPE_BI 1
#define IMAGE_TYPE_BO 2
#define IMAGE_TYPE_AO 3
#define IMAGE_TYPE_AI 4

/* Forward decls */
class CEK9000Device;
class CDeviceMgr;
class CTerminalList;
template <class T>
class CSimpleList;
struct CTerminal;

/* Globals */
extern CDeviceMgr *g_pDeviceMgr;
extern bool g_bDebug;

/* Terminal types */
enum class ETerminalType
{
	UNKNOWN = 0,
	EL10XX,
	EL20XX,
	EL30XX,
	EL40XX,
	EL5XXX,
	EL6XXX,
	EL7XXX,
	EL9XXX,
	ANALOG,
	DIGITAL,
	MAX,
};

#define TERMINAL_FAMILY_ANALOG 0x1
#define TERMINAL_FAMILY_DIGITAL 0x2

/* Errors and error types */
enum EK9KError : int
{
	ERR_OK,
	ERR_GENERIC,
	ERR_NULLPARAM,
	ERR_INVALPARAM,
	ERR_BADID,
	ERR_INVALTERMID,
};

/* Simple struct that holds some data for async processing */
struct STerminalProcessInfo
{
	CALLBACK *m_pCallback;
	CTerminal *m_pTerminal;
	int m_nChannel;
};

void Info(const char* fmt, ...);
void Warning(const char* fmt, ...);
void Error(const char* fmt, ...);

#define DevInfo(fmt, ...) if(g_bDebug) { Info(fmt, __VA_ARGS__); }
#define DevWarn(fmt, ...) if(g_bDebug) { Warning(fmt, __VA_ARGS__); }
#define DevError(fmt, ...) if(g_bDebug) { Error(fmt, __VA_ARGS__); }

/* Strcmp with no case (nc) */
int strcmpnc(const char* str1, const char* str2);
int strncmpnc(const char* str1, const char* str2, int n);

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

public:
	/* Name of record */
	char *m_pRecordName = NULL;
	/* Type */
	ETerminalType m_TerminalType;
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

/* 

class CTerminalList
	Maintains a list of all terminals

TODO: Performance NEEDS to be improved somehow. Possibly use a hash map.

*/
class CTerminalList
{
private:
	class Node
	{
	public:
		Node *m_pNext;
		CTerminal *val;
	};

	Node *m_pRoot = NULL;

public:
public:
	CTerminal *Find(const char *name) const
	{
		if (m_pRoot)
		{
			for (Node *node = m_pRoot; node; node = node->m_pNext)
			{
				if (strcmp(node->val->m_pRecordName, name) == 0)
				{
					return node->val;
				}
			}
		}
		return NULL;
	}

	void Add(CTerminal *term)
	{
		if (!term)
			return;
		if (m_pRoot)
		{
			for (Node *node = m_pRoot; node; node = node->m_pNext)
			{
				if (node->m_pNext == NULL)
				{
					node->m_pNext = new Node();
					node->m_pNext->val = term;
				}
			}
		}
		else
		{
			m_pRoot = new Node();
			m_pRoot->val = term;
		}
	}

	void Remove(const char *name)
	{
		if (m_pRoot)
		{
			for (Node *node = m_pRoot, *prev = NULL; node; prev = node, node = node->m_pNext)
			{
				if (strcmp(node->val->m_pRecordName, name) == 0)
				{
					if (prev)
					{
						prev->m_pNext = node->m_pNext;
						delete node;
					}
					else if (m_pRoot == node)
					{
						m_pRoot = NULL;
						delete node;
					}
					else
						delete node;
				}
			}
		}
	}
};

//==========================================================//
// class CDeviceMgr
//		Manages devices and such
//==========================================================//
class CDeviceMgr
{
private:
	class Node
	{
	public:
		Node *next = NULL;
		CEK9000Device *device = NULL;
	};

	Node *m_pRoot = NULL;

	size_t m_nCount = 0;

	/* for the first/next stuff */
	mutable Node *ctx = NULL;

	/* Mutex for multi-threaded stuff */
	epicsMutexId m_Mutex = 0;

public:
	/* Used to quicky look up terminals */
	CTerminalList m_Terminals;

	friend class CEK9000Device;

public:
	/* Contructor */
	CDeviceMgr();

	/* Destructor */
	~CDeviceMgr();

	/* Initialize CDeviceMgr */
	static int Init();

	void Remove(CEK9000Device *dev);

	int CanAdd(const CEK9000Device &dev);

	void Add(CEK9000Device *dev);

	/* Find device by name */
	CEK9000Device *FindDevice(const char *name) const;

	/* Get first device */
	CEK9000Device *FirstDevice() const;

	CEK9000Device *NextDevice() const;
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
	inline int doEK9000IO(int rw, uint16_t addr, uint16_t len, uint16_t *data);

	/* Do CoE I/O */
	inline int doCoEIO(int rw, uint16_t term, uint16_t index, uint16_t len, uint16_t *data, uint16_t subindex, uint16_t reallen = 0);

	/* same as doEK9000IO except this only operates on the ek9000 itself, term is not used */
	inline int doCouplerIO(int rw, uint16_t term, uint16_t len, uint16_t addr, uint16_t *data, uint16_t subindex = 0);

	/* Reads the value of the CoE register */
	inline int ReadCOECode();

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

	/* Write to output process image */
	/* Length of data should = process image data size */
	int WriteOutput(uint16_t termid, uint16_t *data, int subindex = -1);

	/* Read output process image */
	/* Length of data should = process image data size */
	int ReadOutput(uint16_t termid, uint16_t *data, int subindex = -1);

	/* Read input process image */
	/* Length of data should = process image data size */
	int ReadInput(uint16_t termid, uint16_t *data, int subindex = -1);

	/* Read terminal type */
	int ReadTerminalID(uint16_t termid, uint16_t &termidout);

	/* Poll the ek9000 until data is ready/error */
	/* Return 0 for OK, 1 for error */
	/* Duration is the space between each request, timeout is the number of requests to timeout after */
	inline int Poll(float duration, int timeout);

	/* CoE Interface */

	/* Read device name */
	int CoEReadDeviceName(uint16_t termid, char *outbuf, size_t &outbufsize);

	/* Read version infos */
	int CoEReadVersionInfo(uint16_t termid, char *hardware_ver, size_t &outhardsize, char *software_ver, size_t &outsoftsize);

	/* Read SN */
	int CoEReadSN(uint16_t termid, uint32_t &sn);

	/* Set presentation type, see configuration data in terminal manuals for this */
	int CoESetPresentationType(uint16_t termid, int type);

	/* Read presentation type */
	int CoEReadPresentationType(uint16_t termid, int &type);

	/* Try connect to terminal with CoE */
	/* Returns 1 for connection, 0 for not */
	int CoEVerifyConnection(uint16_t termid);

public:
	/* Process data interaction */

	/* Termid is the term index */
	/* channel is the output number and is 1-based */
	/* data is the value, true for on, false for off */
	int WriteDigitalOut(int termid, int channel, uint16_t data);

	/* termid is the term index */
	/* channel is the output number and is 1-based */
	int WriteAnalogOut(int termid, int channel, uint32_t data);

	/* Reads analog input of the specified terminal */
	/* channel is a 1-based number that represents the signal number */
	int ReadDigitalInput(int termid, int channel, uint16_t& data);

	/* Reads analog output of the specified terminal */
	/* channel is a 1-based number that represents the signal number */
	int ReadAnalogInput(int termid, int channel, uint32_t& data);

	/* Motor out */
	/* TODO: Implement */
	int WriteMotorOut(int termid);

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
};

#include <type_traits>
#include <typeinfo>

template<class...Args>
class iocshFunction
{
	iocshArg* args;
	iocshFuncDef funcdef;
	int nargs;
	std::function<void(Args...)> func;
public:
	iocshFunction(const char* name, std::initializer_list<const char*> list, std::function<void(Args...)> fn) :
		func(fn)
	{
		nargs = list.size();
		if(nargs > 0)
			args = (iocshArg*)malloc(list.size() * sizeof(iocshArg));
		/* Use this to determine types */
		std::vector<Args...> argbuf;
	
		/* Handle all of the types, probably in the worst way possible */
		int i = 0;
		for(auto x : list)
		{
			iocshArgType argtype = getArgType(typeid(argbuf[i]));
			args[i] = {x,argtype};
			i++;
		}
		/* Register the function */
		funcdef = {name, nargs, this->args};
		iocshRegister(&this->funcdef, fn);
	}

	~iocshFunction()
	{
		free(args);
	}

private:
	void wrapper(const iocshArgBuf* argbuf)
	{
	}

	iocshArgType getArgType(const std::type_info& info) const
	{
		if(info == typeid(int) || info == typeid(unsigned))
			return iocshArgInt;
		else if(info == typeid(float) || info == typeid(double) || info == typeid(long double))
			return iocshArgDouble;
		else if(info == typeid(const char*) || info == typeid(char*))
			return iocshArgString;
		else if(info == typeid(pdbbase))
			return iocshArgPdbbase;
		return iocshArgString;
	}
};

#endif
