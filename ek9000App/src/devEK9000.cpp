//======================================================//
// Name: devEK9000.c
// Purpose: Device support for EK9000 and it's associated
// devices
// Authors: Jeremy L.
// Date Created: June 10, 2019
// TODO:
//	-	Run this through valgrind/callgrind/cachegrind to find
//		any leaks or other performance issues
//	-
// Notes:
//	-	Performance could be improved by adding a hashmap
//		type of structure instead of a simple doubly linked
//		list (would use less memory too)
//	-	All device support things are defined and implemented
//		here
//	-	For device specific indicies and such, refer to the docs:
//		EL1XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/el10xx_el11xxen.pdf
//		EL2XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/EL20xx_EL2124en.pdf
//		EL3XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/el30xxen.pdf
//		EL4XXX: https://download.beckhoff.com/download/document/io/ethercat-terminals/el40xxen.pdf
//		EL7XXX:
// Revisions:
//	-	July 15, 2019: Improved error propagation and added
//		new init routines.
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <epicsPrint.h>
#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
#include <boRecord.h>
#include <iocsh.h>
#include <callback.h>
#include <epicsTime.h>
#include <epicsGeneralTime.h>

/* Modbus or asyn includes */
#include <drvModbusAsyn.h>
#include <drvAsynIPPort.h>
#include <modbusInterpose.h>

#include "devEK9000.h"
#include "terminals.h"

#define EK9000_SLAVE_ID 0

/* Some settings */
#define POLL_DURATION 0.05
#define TIMEOUT_COUNT 50

#define printf epicsPrintf
#define print_error epicsPrintf

/* Forward decls */
class CEK9000Device;
class CDeviceMgr;

extern CDeviceMgr *g_pDeviceMgr = 0;

extern "C" epicsThreadId g_PollThread = 0;
extern "C" epicsMutexId g_ThreadMutex = 0;

//==========================================================//
// Utils
//==========================================================//
void PollThreadFunc(void* param);

void Utl_InitThread()
{
	g_ThreadMutex = epicsMutexCreate();
	g_PollThread = epicsThreadCreate("EK9000_PollThread", priorityHigh, 
		epicsThreadGetStackSize(epicsThreadStackMedium), PollThreadFunc, NULL);
}

void PollThreadFunc(void* param)
{
	while(true)
	{
		int status = epicsMutexLock(g_ThreadMutex);
		if(status)
			continue;
		for(CEK9000Device* elem = g_pDeviceMgr->FirstDevice(); elem; elem = g_pDeviceMgr->NextDevice())
		{
			if(elem)
			{
				status = (elem)->Lock();
				if(status)
					continue;
				uint16_t buf = 0;
				(elem)->doEK9000IO(0, 1, 1, &buf);
				(elem)->Unlock();
			}
		}
		epicsThreadSleep(0.5);
		epicsMutexUnlock(g_ThreadMutex);
	}
}

//==========================================================//
// class CTerminal
//		Holds important info about the terminals
//==========================================================//
CTerminal::CTerminal(const CTerminal& other)
{
	this->m_nInputSize = other.m_nInputSize;
	this->m_nOutputSize = other.m_nOutputSize;
	this->m_nInputStart = other.m_nInputStart;
	this->m_nOutputStart = other.m_nOutputStart;
	this->m_pRecordName = strdup(other.m_pRecordName);
	this->m_nTerminalID = other.m_nTerminalID;
	this->m_nTerminalIndex = other.m_nTerminalIndex;
	this->m_TerminalFamily = other.m_TerminalFamily;
}

CTerminal::CTerminal()
{
}

CTerminal::~CTerminal()
{
	if (this->m_pRecordName)
		free(this->m_pRecordName);
}

CTerminal *CTerminal::Create(CEK9000Device *device, uint32_t termid, int termindex, const char *recordname)
{
	CTerminal *term = new CTerminal();
	term->m_pDevice = device;
	int outp = 0, inp = 0;

	/* Create IO areas and set things like record name */
	term->m_pRecordName = strdup(recordname);
	term->m_nTerminalIndex = termindex;
	term->m_nTerminalID = termid;

	if(termid >= 1000 && termid < 3000)
		term->m_TerminalFamily |= TERMINAL_OUTPUT_DIGITAL;
	else if(termid >= 3000 && termid < 8000)
		term->m_TerminalFamily |= TERMINAL_FAMILY_ANALOG;

	/* Get the process image size for this terminal */
	CTerminal::GetTerminalInfo((int)termid, inp, outp);
	term->m_nInputSize = inp;
	term->m_nOutputSize = outp;

	return term;
}

CTerminal *CTerminal::ProcessRecordName(const char *recname, int &outindex, char *outname)
{
	int good = 0;
	char *ret = strdup(recname);
	size_t len = strlen(ret);

	for (int i = len; i >= 0; i--)
	{
		if (ret[i] == ':' && i < len)
		{
			ret[i] = '\0';
			good = 1;
			outindex = atoi(&ret[i + 1]);
		}
	}

	if (!good)
	{
		free(ret);
		return NULL;
	}
	else
	{
		for (CEK9000Device *dev = g_pDeviceMgr->FirstDevice(); dev; dev = g_pDeviceMgr->NextDevice())
		{
			for (int i = 0; i < dev->m_nTerms; i++)
			{
				if (!dev->m_pTerms[i].m_pRecordName)
					continue;
				if (strcmp(dev->m_pTerms[i].m_pRecordName, ret) == 0)
				{
					outname = ret;
					return &dev->m_pTerms[i];
				}
			}
		}
		free(ret);
		return NULL;
	}
}

void CTerminal::GetTerminalInfo(int termid, int &inp_size, int &out_size)
{
	for (int i = 0; i < NUM_TERMINALS; i++)
	{
		if (g_pTerminalInfos[i]->m_nID == termid)
		{
			inp_size = g_pTerminalInfos[i]->m_nInputSize;
			out_size = g_pTerminalInfos[i]->m_nOutputSize;
			return;
		}
	}
}

int CTerminal::doEK9000IO(int type, int startaddr, uint16_t *buf, size_t len)
{
	if(!this->m_pDevice || !this->m_pDevice->m_pDriver)
	{
		return EK_EBADTERM;
	}
	int status = this->m_pDevice->m_pDriver->doModbusIO(0, type, startaddr, buf, len);
	if(status)
	{
		return (status + 0x100);
	}
	return EK_EOK;
}

//==========================================================//
// class CEK9000Device
//		Holds useful vars for interacting with EK9000/EL****
//		hardware
//==========================================================//
CEK9000Device::CEK9000Device()
{
	/* Lets make sure there are no nullptr issues */
	m_pName = (char *)malloc(1);
	m_pName[0] = '\0';
	m_pPortName = (char *)malloc(1);
	m_pPortName[0] = '\0';
	this->m_Mutex = epicsMutexCreate();
}

CEK9000Device::~CEK9000Device()
{
	epicsMutexDestroy(this->m_Mutex);
	if (m_pIP)
		free(m_pIP);
	if (m_pPortName)
		free(m_pPortName);
	if (m_pName)
		free(m_pName);
	for (int i = 0; i < m_nTerms; i++)
		free(m_pTerms[i].m_pRecordName);
	if (m_pTerms)
		free(m_pTerms);
}

CEK9000Device *CEK9000Device::Create(const char *name, const char *ip, int terminal_count)
{
	if (terminal_count < 0 || !name || !ip)
		return NULL;

	CEK9000Device *pek = new CEK9000Device();
	pek->m_nTerms = terminal_count;

	/* Allocate space for terminals */
	pek->m_pTerms = (CTerminal *)malloc(sizeof(CTerminal) * terminal_count);
	memset(pek->m_pTerms, 0, sizeof(CTerminal) * terminal_count);

	/* Free the previously allocated stuff */
	free(pek->m_pPortName);
	free(pek->m_pName);

	/* Copy name */
	pek->m_pName = strdup(name);

	/* Create terminal name */
	size_t prefixlen = strlen(PORT_PREFIX);
	size_t len = strlen(name) + strlen(PORT_PREFIX) + 1;
	pek->m_pPortName = (char *)malloc(len);
	memcpy(pek->m_pPortName, PORT_PREFIX, prefixlen); /* Should be optimized? */
	memcpy(pek->m_pPortName + prefixlen, name, strlen(name) + 1);
	pek->m_pPortName[len - 1] = '\0';

	/* Copy IP */
	pek->m_pIP = strdup(ip);

	int status = drvAsynIPPortConfigure(pek->m_pPortName, ip, 0, 0, 0);

	if (status)
	{
		dbgprintf("Unable to configure IP port.");
		return NULL;
	}

	status = modbusInterposeConfig(pek->m_pPortName, modbusLinkTCP, 5000, 0);

	if (status)
	{
		dbgprintf("Unable to configure modbus driver.");
		return NULL;
	}

	/* check connection */
	asynUser *usr = pasynManager->createAsynUser(NULL, NULL);
	pasynManager->connectDevice(usr, pek->m_pPortName, 0);
	int conn = 0;
	pasynManager->isConnected(usr, &conn);
	pasynManager->disconnect(usr);
	pasynManager->freeAsynUser(usr);

	if (!conn)
	{
		dbgprintf("Error while connecting to device %s.", name);
		return NULL;
	}

	pek->m_pDriver = new drvModbusAsyn(pek->m_pName, pek->m_pPortName,
									   0, 2, -1, 256, dataTypeUInt16, 150, "");

	/* Disable wdt for now */
	uint16_t buf = 2;
	pek->m_pDriver->doModbusIO(0, MODBUS_WRITE_SINGLE_REGISTER, 0x1122, &buf, 1);

	g_pDeviceMgr->Add(pek);
	return pek;
}

int CEK9000Device::AddTerminal(const char *name, int type, int position)
{
	if (position > m_nTerms || !name)
		return EK_EBADPARAM;

	CTerminal *term = CTerminal::Create(this, type, position, name);

	if (term)
	{
		memcpy(&this->m_pTerms[position -1], term, sizeof(CTerminal));
		return EK_EOK;
	}
	return EK_EERR;
}

/* Verifies that terminals have the correct ID */
/* Sets the process image size */
int CEK9000Device::InitTerminal(int term)
{
	if (term < 0 || term >= m_nTerms)
		return ERR_INVALPARAM;

	/* Read ther terminal's id */
	uint16_t tid = 0;
	this->ReadTerminalID(term, tid);

	/* Verify that the terminal has the proper id */
	CTerminal *terminal = &this->m_pTerms[term];

	if (tid != terminal->m_nTerminalID)
		return EK_ETERMIDMIS;

	return ERR_OK;
}

/* This will configure process image locations in each terminal */
/* It will also verify that terminals have the correct type (reads terminal type from the device then yells if its not 
the same as the user specified.) */
int CEK9000Device::InitTerminals()
{
	if (m_bInit)
	{
		printf("CEK9000Device: Already initialized.\n");
		return 1;
	}
	m_bInit = true;

	/* Figure out the register map */
	int coil_in = 1, coil_out = 1;
	int reg_in = 1, reg_out = 0x800;
	/* in = holding regs, out = inp regs */
	/* analog terms are mapped FIRST */
	/* then digital termas are mapped */
	/* holding regsiters can have bit offsets */
	/* First loop: analog terms */
	for (int i = 0; i < this->m_nTerms; i++)
	{
		CTerminal& term = m_pTerms[i];
		if ((term.m_TerminalFamily | TERMINAL_FAMILY_ANALOG) == TERMINAL_FAMILY_ANALOG)
		{
			term.m_nInputStart = reg_in;
			term.m_nOutputStart = reg_out;
			reg_in += term.m_nInputSize;
			reg_out += term.m_nOutputSize;
		}
	}

	/* Second loop: digital terms */
	for (int i = 0; i < this->m_nTerms; i++)
	{
		CTerminal *term = &m_pTerms[i];
		if ((term->m_TerminalFamily | TERMINAL_FAMILY_DIGITAL) == TERMINAL_FAMILY_DIGITAL)
		{
			term->m_nInputStart = coil_in;
			term->m_nOutputStart = coil_out;
			coil_in += term->m_nInputSize;
			coil_out += term->m_nOutputSize;
		}
	}
	return EK_EOK;
}

CTerminal *CEK9000Device::GetTerminal(const char *recordname)
{
	for (int i = 0; i < m_nTerms; i++)
	{
		if (strcmp(m_pTerms[i].m_pRecordName, recordname) == 0)
			return &m_pTerms[i];
	}
	return NULL;
}

int CEK9000Device::VerifyConnection() const
{
	/* asynUsers should be pretty cheap to create */
	asynUser *usr = pasynManager->createAsynUser(NULL, NULL);
	usr->timeout = 0.5; /* 500ms timeout */

	/* Try for connection */
	pasynManager->connectDevice(usr, this->m_pPortName, 0);
	int yn = 0;
	pasynManager->isConnected(usr, &yn);
	pasynManager->disconnect(usr);

	pasynManager->freeAsynUser(usr);

	return yn;
}

int CEK9000Device::CoEVerifyConnection(uint16_t termid)
{
	uint16_t dat;
	if (this->doCoEIO(0, (uint16_t)termid, 1008, 1, &dat, 0) != EK_EOK)
	{
		return 0;
	}
	return 1;
}

void CEK9000Device::ReportError(int errcode, const char* _msg)
{
	const char* msg = this->ErrorToString(errcode);
	const char* __msg = "";
	if(_msg)
		__msg = _msg;
	if(m_bDebug)
	{
		time_t time;
		tm _tm;
		epicsTime_localtime(&time, &_tm);
		epicsStdoutPrintf("%i/%i/%i %i:%i:%i CEK9000Device: %s: Error Ocurred: error=%x message=%s\n",
			_tm.tm_year, _tm.tm_mon, _tm.tm_mday, _tm.tm_hour, _tm.tm_min, _tm.tm_sec, __msg,
			errcode, msg);
	}
}

int CEK9000Device::doCoEIO(int rw, uint16_t term, uint16_t index, uint16_t len, uint16_t *data, uint16_t subindex)
{
	/* write */
	if (rw)
	{
		uint16_t tmp_data[] =
			{
				1,			/* 0x1400 = execute */
				term,		/* 0x1401 = term id */
				index,		/* 0x1402 = Le object */
				subindex, 	/* 0x1403 = subindex */
				len,		/* 0x1404 = length */
			};
		/* Prepare buffer */
		uint16_t *buf = (uint16_t *)malloc(sizeof(uint16_t) * (6 + len));
		memcpy(buf, tmp_data, sizeof(uint16_t) * 5);
		memcpy((buf + 5), data, sizeof(uint16_t) * len);

		this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1400, tmp_data, len + 5);

		if (this->Poll(0.05, 50))
		{
			uint16_t dat = 0;
			this->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1400, &dat, 1);
			if (dat != 0)
			{
				data[0] = dat;
				return EK_EADSERR;
			}
			return EK_EERR;
		}
		return EK_EOK;
	}
	/* read */
	else
	{
		uint16_t tmp_data[] =
			{
				1,		  /* 0x1400 = exec */
				term,	 /* 0x1401 = term id */
				index,	/* 0x1402 = obj */
				subindex, /* 0x1403 = subindex */
				0,		  /* 0x1404 = len = 0 */
			};

		/* tell it what to do */
		this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1400, tmp_data, 5);

		/* poll */
		if (this->Poll(POLL_DURATION, TIMEOUT_COUNT))
		{
			uint16_t dat = 0;
			this->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1405, &dat, 1);
			if (dat != 0)
			{
				data[0] = dat;
				return EK_EADSERR;
			}
			return EK_EERR;
		}

		/* read result */
		this->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1406, data, len);
		return EK_EOK;
	}
}

int CEK9000Device::doCouplerIO(int rw, uint16_t term, uint16_t len, uint16_t addr, uint16_t *data, uint16_t subindex)
{
	int status = epicsMutexLock(m_Mutex);
	if(status != epicsMutexLockOK)
	{
		if(status == epicsMutexLockTimeout)
			return EK_EMUTEXTIMEOUT;
		return EK_EBADMUTEX;
	}
	
	/* write */
	if (rw)
	{
		status = this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, addr, data, len);
		if(status)
		{
			epicsMutexUnlock(m_Mutex);
			return status + 0x100;
		}
		return EK_EOK;
	}
	else
	{
		status = this->m_pDriver->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, addr, data, len);
		if(status)
		{
			epicsMutexUnlock(m_Mutex);
			return status + 0x100;
		}
		return EK_EOK;
	}
}

int CEK9000Device::doEK9000IO(int rw, uint16_t addr, uint16_t len, uint16_t *data)
{
	int status = 0;
	/* write */
	if (rw)
	{
		status = this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, addr, data, len);
		if(status)
		{
			return status + 0x100;
		}
		return EK_EOK;
	}
	/* read */
	else
	{
		status = this->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, addr, data, len);
		if(status)
		{
			return status + 0x100;
		}
		return EK_EOK;
	}
	return EK_EBADPARAM;
}

/* Read terminal type */
int CEK9000Device::ReadTerminalType(uint16_t termid, int &id)
{
	uint16_t dat = 0;
	this->doEK9000IO(0, 0x6000, 1, &dat);
	id = dat;
	return EK_EOK;
}

/* Reads coupler id */
int CEK9000Device::ReadCouplerID(char *outbuf, size_t &outbufsize)
{
	if (!outbuf || outbufsize < sizeof(uint16_t) * 7 + 1)
		return EK_EBADPARAM;

	/* Coupler id len is 6 words */
	memset(outbuf, 0, sizeof(uint16_t) * 7 + 1);
	uint16_t id[7];
	int status = this->doEK9000IO(0, 0x1008, 7, id);

	if (!status)
	{
		memcpy(outbuf, id, sizeof(uint16_t) * 7);
		outbufsize = sizeof(uint16_t) * 7 + 1;
		return EK_EOK;
	}
	this->m_nError = id[0];
	return status;
}

/* Reads the length of the process image */
int CEK9000Device::ReadProcessImageSize(uint16_t &anal_out, uint16_t &anal_in, uint16_t &dig_out, uint16_t &dig_in)
{
	uint16_t image[4];
	int status = this->doEK9000IO(0, 0x1010, 4, image);
	if (!status)
	{
		anal_out = image[0];
		anal_in = image[1];
		dig_out = image[2];
		dig_in = image[3];
		return EK_EOK;
	}
	this->m_nError = image[0];
	return status;
}

/* Reads the current watchdog time */
int CEK9000Device::ReadWatchdogTime(uint16_t &out)
{
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1020, 1, &tmp);
	if (!stat)
	{
		out = tmp;
		return EK_EOK;
	}
	this->m_nError = tmp;
	return stat;
}

/* Read the number of fallbacks triggered */
int CEK9000Device::ReadNumFallbacksTriggered(uint16_t &out)
{
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1021, 1, &tmp);
	if (!stat)
	{
		out = tmp;
		return EK_EOK;
	}
	this->m_nError = tmp;
	return stat;
}

/* Read the number of tcp connections */
int CEK9000Device::ReadNumTCPConnections(uint16_t &out)
{
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1022, 1, &tmp);
	if (!stat)
	{
		out = tmp;
		return EK_EOK;
	}
	this->m_nError = tmp;
	return stat;
}

/* Read hardware/software versions */
int CEK9000Device::ReadVersionInfo(uint16_t &hardver, uint16_t &softver_major, uint16_t &softver_minor, uint16_t &softver_patch)
{
	uint16_t ver[4];
	int status = this->doEK9000IO(0, 0x1030, 4, ver);
	if (!status)
	{
		hardver = ver[0];
		softver_major = ver[1];
		softver_minor = ver[2];
		softver_patch = ver[3];
		return EK_EOK;
	}
	this->m_nError = ver[0];
	return status;
}

/* Read the serial number */
int CEK9000Device::ReadSerialNumber(uint16_t &sn)
{
	uint16_t tmp = 0;
	int stat = this->doEK9000IO(0, 0x1034, 1, &tmp);
	if (!stat)
	{
		sn = tmp;
		return EK_EOK;
	}
	this->m_nError = tmp;
	return stat;
}

/* Read manfacturing date */
int CEK9000Device::ReadMfgDate(uint16_t &day, uint16_t &mon, uint16_t &year)
{
	uint16_t date[3];
	int status = this->doEK9000IO(0, 0x1035, 3, date);
	if (!status)
	{
		day = date[0];
		mon = date[1];
		year = date[2];
		return EK_EOK;
	}
	this->m_nError = date[0];
	return status;
}

/* Read EBus status */
int CEK9000Device::ReadEBusStatus(uint16_t &status)
{
	ushort tmp = 0; // to store tmp data
	int stat = this->doEK9000IO(0, 0x1040, 1, &tmp);
	if (!stat)
	{
		status = tmp;
		return EK_EOK;
	}
	this->m_nError = tmp;
	return stat;
}

/* Write the watcdog time */
int CEK9000Device::WriteWatchdogTime(uint16_t time)
{
	return this->doEK9000IO(1, 0x1120, 1, &time);
}

/* Reset watchdog timer */
int CEK9000Device::WriteWatchdogReset()
{
	uint16_t data = 1;
	//return this->doEK9000IO(1, 0, 1, 0x1121, &data);
	this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1121, &data, 1);
	return EK_EOK;
}

/* Write watchdog type */
int CEK9000Device::WriteWatchdogType(uint16_t type)
{
	return this->doEK9000IO(1, 0x1122, 1, &type);
}

/* Write fallback mode */
int CEK9000Device::WriteFallbackMode(uint16_t mode)
{
	return this->doEK9000IO(1, 0x1123, 1, &mode);
}

/* Enable/disable writing to second modbus client */
int CEK9000Device::WriteWritelockMode(uint16_t mode)
{
	return this->doEK9000IO(1, 0x1124, 1, &mode);
}

/* Read terminal ID */
int CEK9000Device::ReadTerminalID(uint16_t termid, uint16_t &out)
{
	uint16_t tmp = 0;
	this->m_pDriver->doModbusIO(0, MODBUS_READ_INPUT_REGISTERS, 0x6000 + (termid), &tmp, 1);
	if (tmp == 0)
		return EK_ENOCONN;
	out = tmp;
	return EK_EOK;
}

int CEK9000Device::SetOption(int term, const char *opt, const char *val)
{
	if (!opt && !val)
		return EK_EBADPARAM;
	if (term < 0 || term > this->m_nTerms)
		return EK_EBADTERM;

	if (strcmp(opt, "WatchdogTimer") == 0)
	{
		int i = atoi(val);
		if (i >= 0 && i <= 1000)
			return this->WriteWatchdogTime((uint16_t)i);
		else
			return EK_EBADPARAM;
	}
	else if (strcmp(opt, "WatchdogType") == 0)
	{
		int i = atoi(val);
		if (i >= 0 && i <= 2)
			return this->WriteWatchdogType(i);
		else
			return EK_EBADPARAM;
	}
	else if (strcmp(opt, "FallbackMode") == 0)
	{
		int i = atoi(val);
		if (i >= 0 && i <= 2)
			return this->WriteFallbackMode(i);
		else
			return EK_EBADPARAM;
	}
	else if (strcmp(opt, "WriteLock") == 0)
	{
		int i = atoi(val);
		if (i >= 0 && i <= 1)
			return this->WriteWritelockMode(i);
		else
			return EK_EBADPARAM;
	}
	return EK_EOK;
}

int CEK9000Device::Poll(float duration, int timeout)
{
	ushort dat = 0;
	do
	{
		this->m_pDriver->doModbusIO(EK9000_SLAVE_ID, MODBUS_READ_HOLDING_REGISTERS, 0x1400, &dat, 1);
		epicsThreadSleep(duration);
		timeout--;
	} while ((dat | 0x200) == 0x200 && timeout > 0);

	if (timeout == 0)
		return 1;
	else
		return 0;
}

int CEK9000Device::LastError()
{
	int tmp = m_nError;
	m_nError = EK_EOK;
	return tmp;
}

const char *CEK9000Device::LastErrorString()
{
	return ErrorToString(LastError());
}

const char *CEK9000Device::ErrorToString(int i)
{
	switch (i)
	{
	case EK_EOK:
		return "No Error";
	case EK_EBADIP:
		return "Invalid IP format";
	case EK_EBADPARAM:
		return "Invalid parameter";
	case EK_EBADPORT:
		return "Invalid port number";
	case EK_EBADPTR:
		return "Invalid pointer";
	case EK_EBADTERM:
		return "Invalid terminal";
	case EK_EBADTYP:
		return "Invalid terminal type";
	case EK_EERR:
		return "Unspecified error";
	case EK_ENOCONN:
		return "Bad connection";
	case EK_ENODEV:
		return "Invalid device";
	case EK_ENOENT:
		return "No directory entry";
	case EK_EWTCHDG:
		return "Watchdog error";
	default:
		return "Invalid error code";
	}
}

//==========================================================//
// class CDeviceMgr (impl)
//		Manages devices and such
//==========================================================//
CDeviceMgr::CDeviceMgr()
{
	this->m_Mutex = epicsMutexCreate();
}

CDeviceMgr::~CDeviceMgr()
{
	if(this->m_Mutex)
		epicsMutexDestroy(this->m_Mutex);
}

int CDeviceMgr::Init()
{
	g_pDeviceMgr = new CDeviceMgr();
	return 0;
}

int CDeviceMgr::CanAdd(const CEK9000Device &dev)
{
	int status = epicsMutexLock(m_Mutex);	
	if(status)
		return 0;
	int ret = (this->FindDevice(dev.m_pName) == NULL);
	epicsMutexUnlock(m_Mutex);
	return ret;
}

CEK9000Device *CDeviceMgr::FindDevice(const char *name) const
{
	int status = epicsMutexLock(m_Mutex);
	if(status)
		return NULL;

	if (!name)
	{
		epicsMutexUnlock(m_Mutex);
		return NULL;
	}

	if (m_pRoot)
	{
		for (Node *node = m_pRoot; node; node = node->next)
		{
			if (strcmp(name, node->device->m_pName) == 0)
			{
				epicsMutexUnlock(m_Mutex);
				return node->device;
			}
		}
	}
	epicsMutexUnlock(m_Mutex);
	return NULL;
}

void CDeviceMgr::Add(CEK9000Device *dev)
{
	int status = epicsMutexLock(m_Mutex);
	if(status)
		return;

	if (!dev)
	{
		epicsMutexUnlock(m_Mutex);
		return;
	}

	if (!m_pRoot)
	{
		m_pRoot = new Node();
		m_pRoot->device = dev;
		m_nCount++;
		epicsMutexUnlock(m_Mutex);
		return;
	}
	else
	{
		for (Node *node = m_pRoot; node; node = node->next)
		{
			if (!node->next)
			{
				node->next = new Node();
				node->next->device = dev;
				m_nCount++;
				epicsMutexUnlock(m_Mutex);
				return;
			}
		}
	}
	epicsMutexUnlock(m_Mutex);
}

void CDeviceMgr::Remove(CEK9000Device *dev)
{
	int status = epicsMutexLock(m_Mutex);
	if(status)
		return;

	if (!dev)
	{
		epicsMutexUnlock(m_Mutex);
		return;
	}

	if (m_pRoot)
	{
		for (Node *node = m_pRoot, *prev = NULL; node; prev = node, node = node->next)
		{
			if (strcmp(dev->m_pName, node->device->m_pName) == 0)
			{
				if (prev)
				{
					prev->next = node->next;
					delete node;
					m_nCount--;
					epicsMutexUnlock(m_Mutex);
					return;
				}
				/* This must be root node */
				else
				{
					delete m_pRoot;
					m_nCount--;
					epicsMutexUnlock(m_Mutex);
					return;
				}
			}
		}
	}
	epicsMutexUnlock(m_Mutex);
}

CEK9000Device *CDeviceMgr::FirstDevice() const
{
	/* TODO: Is locking really needed here? */
	int status = epicsMutexLock(m_Mutex);
	if(status)
		return NULL;
	
	this->ctx = m_pRoot;
	
	if (ctx)
	{
		CEK9000Device* dev = ctx->device;
		epicsMutexUnlock(m_Mutex);
		return dev;
	}
	else
	{
		epicsMutexUnlock(m_Mutex);
		return NULL;
	}
}

CEK9000Device *CDeviceMgr::NextDevice() const
{
	int status = epicsMutexLock(m_Mutex);
	if(status)
		return NULL;

	if (!ctx)
	{
		epicsMutexUnlock(m_Mutex);
		return NULL;
	}
	this->ctx = ctx->next;

	if (ctx)
	{
		CEK9000Device* dev = ctx->device;
		epicsMutexUnlock(m_Mutex);
		return dev;
	}
	else
	{
		epicsMutexUnlock(m_Mutex);
		return NULL;
	}
}

//==========================================================//
// IOCsh functions here
//==========================================================//
void ek9000Configure(const iocshArgBuf *args)
{
	const char *name = args[0].sval;
	const char *ip = args[1].sval;
	int port = args[2].ival;
	int num = args[3].ival;

	if (!name)
	{
		epicsStdoutPrintf("Invalid name passed.\n");
		return;
	}

	if (!ip)
	{
		epicsStdoutPrintf("Invalid IP passed.\n");
		return;
	}

	if (num < 0)
	{
		epicsStdoutPrintf("Invalid terminal count passed.\n");
		return;
	}

	if (port <= 0)
	{
		epicsStdoutPrintf("The port %i is invalid.\n", port);
		return;
	}

	CEK9000Device *dev;

	char ipbuf[64];
	sprintf(ipbuf, "%s:%i", ip, port);

	dev = CEK9000Device::Create(name, ipbuf, num);

	if (!dev)
	{
		epicsStdoutPrintf("Unable to create device: Unspecified error.\n");
		return;
	}

	g_pDeviceMgr->Add(dev);
}

void ek9000ConfigureTerminal(const iocshArgBuf *args)
{
	const char *ek = args[0].sval;
	const char *name = args[1].sval;
	const char *type = args[2].sval;
	int id = args[3].ival;

	if (!ek || !name || !type || id < 0)
	{
		epicsStdoutPrintf("Invalid parameter passed!\n");
		return;
	}

	CEK9000Device *dev = g_pDeviceMgr->FindDevice(ek);

	/* If we cant find the device :( */
	if (!dev)
	{
		epicsStdoutPrintf("Unable to create terminal \"%s\": Device by the name of \"%s\" not found.\n",
						  name, ek);
		return;
	}

	int tid = 0;
	for(int i = 0; i < NUM_TERMINALS; i++)
	{
		if(strcmp(g_pTerminalInfos[i]->m_pString, type) == 0)
		{
			tid = g_pTerminalInfos[i]->m_nID;
			break;
		}
	}
	if(tid == 0) 
	{
		epicsStdoutPrintf("Unabel to create terminal %s: No terminal with the ID %s found.\n",
			name, type);
		return;
	}

	/* Check for out of boundries */
	if (id > dev->m_nTerms)
	{
		epicsStdoutPrintf("Unable to create terminal \"%s\": Terminal index out of range.\n", name);
		return;
	}

	int status = dev->AddTerminal(name, tid, id);
	
	if (status)
	{
		dbgprintf("Failed to create terminal.");
		return;
	}

	epicsStdoutPrintf("Created terminal \"%s\"\n", name);
}

void ek9000SetOption(const iocshArgBuf *args)
{
	const char *ek9k = args[0].sval;
	int term = args[1].ival;
	const char *opt = args[2].sval;
	const char *val = args[3].sval;

	if (!opt || !val || !ek9k)
	{
		epicsStdoutPrintf("Unable to set option: Invalid parameter passed.\n");
		return;
	}

	CEK9000Device *dev = g_pDeviceMgr->FindDevice(ek9k);

	if (!dev)
	{
		epicsStdoutPrintf("Invalid device.\n");
		return;
	}

	dev->SetOption(term, opt, val);
}

void ek9000Stat(const iocshArgBuf *args)
{
	const char *ek9k = args[0].sval;
	if (!ek9k)
	{
		epicsStdoutPrintf("Invalid parameter.\n");
		return;
	}
	CEK9000Device *dev = g_pDeviceMgr->FindDevice(ek9k);

	if (!dev)
	{
		printf("Invalid device.\n");
		return;
	}

	if (!dev->VerifyConnection())
	{
		printf("Device is not connected.\n");
		return;
	}

	uint16_t ao = 0, ai = 0, bo = 0, bi = 0, tcp = 0, sn = 0, wtd = 0;
	uint16_t hver = 0, svermaj = 0, svermin = 0, sverpat = 0;
	uint16_t day = 0, month = 0, year = 0;

	if (dev->ReadProcessImageSize(ao, ai, bo, bi))
	{
		printf("Error while reading process image size.\n");
		return;
	}

	if (dev->ReadNumTCPConnections(tcp))
	{
		printf("Error while reading tcp connection count.\n");
		return;
	}

	if (dev->ReadSerialNumber(sn))
	{
		printf("Error while reading serial number.\n");
		return;
	}

	if (dev->ReadVersionInfo(hver, svermaj, svermin, sverpat))
	{
		printf("Error while reading version info\n");
		return;
	}

	if (dev->ReadNumFallbacksTriggered(wtd))
	{
		printf("Error while reading number of fallbacks triggered.\n");
		return;
	}

	if (dev->ReadMfgDate(day, month, year))
	{
		printf("Error while reading mfg date.\n");
		return;
	}

	printf("Device: %s\n", ek9k);
	printf("AO size: %u\n", ao);
	printf("AI size: %u\n", ai);
	printf("BI size: %u\n", bi);
	printf("BO size: %u\n", bo);
	printf("TCP connections: %u\n", tcp);
	printf("Serial number: %u\n", sn);
	printf("Hardware Version: %u\n", hver);
	printf("Software Version: %u.%u.%u\n", svermaj, svermin, sverpat);
	printf("Fallbacks triggered: %u\n", wtd);
	printf("Mfg date: %u/%u/%u\n", month, day, year);
}

void ek9000DoTest(const iocshArgBuf *args)
{
}

void ek9000EnableDebug(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	if(!ek9k)
	{
		epicsStdoutPrintf("Invalid parameter!\n");
		return;
	}
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);
	if(!dev)
	{
		epicsStdoutPrintf("No device by name: %s\n", ek9k);
		return;
	}
	dev->EnableDebug(true);
}

void ek9000DisableDebug(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	if(!ek9k)
	{
		epicsStdoutPrintf("Invalid parameter!\n");
		return;
	}
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);
	if(!dev)
	{
		epicsStdoutPrintf("No device by name: %s\n", ek9k);
		return;
	}
	dev->EnableDebug(false);
}

int ek9000RegisterFunctions()
{
	/* ek9000Configure(name, ip, termcount) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"IP", iocshArgString};
		static const iocshArg arg3 = {"Port", iocshArgInt};
		static const iocshArg arg4 = {"# of Terminals", iocshArgInt};
		static const iocshArg *const args[] = {&arg1, &arg2, &arg3, &arg4};
		static const iocshFuncDef func = {"ek9000Configure", 4, args};
		iocshRegister(&func, ek9000Configure);
	}

	/* ek9000ConfigureTerminal(ek9000, name, type, position) */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg2 = {"Record Name", iocshArgString};
		static const iocshArg arg3 = {"Type", iocshArgString};
		static const iocshArg arg4 = {"Positon", iocshArgInt};
		static const iocshArg *const args[] = {&arg1, &arg2, &arg3, &arg4};
		static const iocshFuncDef func = {"ek9000ConfigureTerminal", 4, args};
		iocshRegister(&func, ek9000ConfigureTerminal);
	}

	/* ek9000SetOption(ek9000, opt, value) */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg arg2 = {"Option", iocshArgString};
		static const iocshArg arg3 = {"Value", iocshArgString};
		static const iocshArg *const args[] = {&arg1, &arg2, &arg3};
		static const iocshFuncDef func = {"ek9000SetOption", 3, args};
		iocshRegister(&func, ek9000SetOption);
	}

	/* ek9000Stat */
	{
		static const iocshArg arg1 = {"EK9k", iocshArgString};
		static const iocshArg *const args[] = {&arg1};
		static const iocshFuncDef func = {"ek9000Stat", 1, args};
		iocshRegister(&func, ek9000Stat);
	}

	/* ek9000EnableDebug */
	{
		static const iocshArg arg1 = {"EK9k", iocshArgString};
		static const iocshArg *const args[] = {&arg1};
		static const iocshFuncDef func = {"ek9000EnableDebug", 1, args};
		iocshRegister(&func, ek9000EnableDebug);
	}

	/* ek9000DisableDebug */
	{
		static const iocshArg arg1 = {"EK9K", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"ek9000DisableDebig", 1, args};
		iocshRegister(&func, ek9000DisableDebug);
	}

	CDeviceMgr::Init();

	return 0;
}
epicsExportRegistrar(ek9000RegisterFunctions);

//======================================================//
//
// Fake device support module so that we can just do some
// initialization for the ek9000
//
//======================================================//
static long ek9000_init(int);
static long ek9000_init_record(void *prec);

struct
{
	long number;
	DEVSUPFUN dev_report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN get_ioint_info;
	DEVSUPFUN write_record;
} devEK9000 = {
	5,
	NULL,
	(DEVSUPFUN)ek9000_init,
	(DEVSUPFUN)ek9000_init_record,
	NULL,
	NULL,
};

epicsExportAddress(dset, devEK9000);

static long ek9000_init(int after)
{
	if (after == 0)
	{
		epicsStdoutPrintf("Initializing EK9000 Couplers.\n");
		for (CEK9000Device *dev = g_pDeviceMgr->FirstDevice(); dev; dev = g_pDeviceMgr->NextDevice())
		{
			if (!dev->m_bInit)
				dev->InitTerminals();
		}
		epicsStdoutPrintf("Initialization Complete.\n");
		Utl_InitThread();
	}
	return 0;
}

static long ek9000_init_record(void *prec)
{
	epicsStdoutPrintf("FATAL ERROR: You should not use devEK9000 on any records!\n");
	epicsAssert(__FILE__, __LINE__, "FATAL ERROR: You should not use devEK9000 on any records!\n", "Jeremy L.");
	return 0;
}