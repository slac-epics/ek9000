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
//	-	Feb 7, 2020: Implemented record-based CoE configuration
//	-	Feb 7, 2020: doCoEIO will now set an errno variable
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
#include <errno.h>

/* Record includes */
#include <biRecord.h>
#include <boRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <int64inRecord.h>
#include <int64outRecord.h>

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

/* Forward decls */
class CEK9000Device;
class CDeviceMgr;

/* Globals */
CDeviceMgr *g_pDeviceMgr = 0;
bool g_bDebug = false;
epicsThreadId g_PollThread = 0;
epicsMutexId g_ThreadMutex = 0;
int g_nPollDelay = 250;

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
		for(CEK9000Device* elem = g_pDeviceMgr->FirstDevice(); elem; elem = g_pDeviceMgr->NextDevice())
		{
			if(elem)
			{
				int status = elem->Lock();
				/* check connection */
				bool connected = elem->VerifyConnection();
				if(!connected && elem->m_bConnected)
				{
					Warning("%s: Link status changed to DISCONNECTED\n", elem->m_pName);
					elem->m_bConnected = false;
				}
				if(connected && !elem->m_bConnected)
				{
					Warning("%s: Link status changed to CONNECTED\n", elem->m_pName);
					elem->m_bConnected = true;
				}
				if(status)
					continue;
				uint16_t buf = 1;
				elem->m_pDriver->doModbusIO(0, MODBUS_WRITE_SINGLE_REGISTER, 0x1121, &buf, 1);
				elem->Unlock();
			}
		}
		epicsThreadSleep(g_nPollDelay/1000.0f);
	}
}

void Info(const char* fmt, ...)
{
	time_t clk = time(0);
	tm _tm;
	epicsTime_localtime(&clk, &_tm);
	epicsPrintf("%i:%i ", _tm.tm_hour, _tm.tm_min);
	va_list list;
	va_start(list, fmt);
	epicsVprintf(fmt, list);
	va_end(list);
}

void Warning(const char* fmt, ...)
{	
	epicsTimeStamp stmp;
	epicsTimeGetCurrent(&stmp);
	char txt[40];
	epicsTimeToStrftime(txt, 40, "%Y/%m/%d %H:%M:%S.%03f ", &stmp);
	epicsPrintf("%s", txt);
	va_list list;
	va_start(list, fmt);
	epicsVprintf(fmt, list);
	va_end(list);
}

void Error(const char* fmt, ...)
{
	epicsTimeStamp stmp;
	epicsTimeGetCurrent(&stmp);
	char txt[40];
	epicsTimeToStrftime(txt, 40, "%Y/%m/%d %H:%M:%S.%03f ", &stmp);
	epicsPrintf("%s", txt);
	va_list list;
	va_start(list, fmt);
	epicsVprintf(fmt, list);
	va_end(list);
}

int strcmpnc(const char* str1, const char* str2)
{
	size_t str1l = strlen(str1);
	size_t str2l = strlen(str2);
	for(int i = 0; str1[i] && str2[i]; i++)
	{
		if(tolower(str1[i]) != tolower(str2[i]))
			return 1;
	}
	if(str1l < str2l) return -1;
	if(str2l < str1l) return 1;
	return 0;
}

int strncmpnc(const char* str1, const char* str2, int n)
{
	for(int i = 0; i < n; i++)
	{
		if(tolower(str1[i]) != tolower(str2[i])) return 1;
	}
	return 0;
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
		term->m_TerminalFamily = TERMINAL_FAMILY_DIGITAL;
	else if(termid >= 3000 && termid < 8000)
		term->m_TerminalFamily = TERMINAL_FAMILY_ANALOG;

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
		if (ret[i] == ':' && (size_t)i < len)
		{
			ret[i] = '\0';
			good = 1;
			outindex = atoi(&ret[i + 1]);
			break;
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
		if (g_pTerminalInfos[i]->m_nID == (uint32_t)termid)
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
	this->queue = new epicsMessageQueue(500, sizeof(void*));
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
		Error("CEK9000Device::Create(): Unable to configure drvAsynIPPort.");
		return NULL;
	}

	status = modbusInterposeConfig(pek->m_pPortName, modbusLinkTCP, 5000, 0);

	if (status)
	{
		Error("CEK9000Device::Create(): Unable to configure modbus driver.");
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
		Error("CEK9000Device::Create(): Error while connecting to device %s.", name);
		return NULL;
	}

	pek->m_pDriver = new drvModbusAsyn(pek->m_pName, pek->m_pPortName,
									   0, 2, -1, 256, dataTypeUInt16, 150, "");

	/* wdt =  */
	uint16_t buf = 1;
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

int CEK9000Device::Lock()
{
	return this->m_pDriver->lock();
}

void CEK9000Device::Unlock()
{
	this->m_pDriver->unlock();
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
		epicsPrintf("CEK9000Device: Already initialized.\n");
		return 1;
	}
	m_bInit = true;

	/* Figure out the register map */
	int coil_in = 1, coil_out = 1;
	int reg_in = 0, reg_out = 0x800;
	/* in = holding regs, out = inp regs */
	/* analog terms are mapped FIRST */
	/* then digital termas are mapped */
	/* holding regsiters can have bit offsets */
	/* First loop: analog terms */
	for (int i = 0; i < this->m_nTerms; i++)
	{
		CTerminal* term = &m_pTerms[i];
		if (term->m_TerminalFamily == TERMINAL_FAMILY_ANALOG)
		{
			term->m_nInputStart = reg_in;
			term->m_nOutputStart = reg_out;
			reg_in += term->m_nInputSize;
			reg_out += term->m_nOutputSize;
		}
	}

	/* Second loop: digital terms */
	for (int i = 0; i < this->m_nTerms; i++)
	{
		CTerminal *term = &m_pTerms[i];
		if (term->m_TerminalFamily == TERMINAL_FAMILY_DIGITAL)
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

/* 1 for write and 0 for read */
/* LENGTH IS IN REGISTERS */
int CEK9000Device::doCoEIO(int rw, uint16_t term, uint16_t index, uint16_t len, uint16_t *data, uint16_t subindex, uint16_t reallen)
{
	/* write */
	if (rw)
	{
		uint16_t tmp_data[512] = 
		{
			1,
			(uint16_t)(term | 0x8000), /* Bit 15 needs to be 1 to indicate a write */
			index,
			subindex,
			reallen ? reallen : (uint16_t)(len*2),
			0, /* Error code */
		};

		memcpy(tmp_data+6, data, len*sizeof(uint16_t));
		this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1400, tmp_data, len+7);
		if (!this->Poll(0.005, TIMEOUT_COUNT))
		{
			this->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1400, tmp_data, 6);
			/* Write tmp data */
			if((tmp_data[0] & 0x400) != 0x400) {
				LastADSErr = tmp_data[5];
				return EK_EADSERR;
			}
		}
		else
			return EK_EERR;
		return EK_EOK;
	}
	/* read */
	else
	{
		uint16_t tmp_data[512] =
			{
				1,		  	/* 0x1400 = exec */
				term,	 	/* 0x1401 = term id */
				index,		/* 0x1402 = obj */
				subindex, 	/* 0x1403 = subindex */
				0,	  		/* 0x1404 = len = 0 */
				0,
				0,
				0,
				0,
			};

		/* tell it what to do */
		this->m_pDriver->doModbusIO(0, MODBUS_WRITE_MULTIPLE_REGISTERS, 0x1400, tmp_data, 9);

		/* poll */
		if (this->Poll(0.005, TIMEOUT_COUNT))
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
		epicsThreadSleep(0.05);
		/* read result */
		int res = this->m_pDriver->doModbusIO(0, MODBUS_READ_HOLDING_REGISTERS, 0x1406, data, len);
		if(res)
			return EK_EERR;
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

int CEK9000Device::Poll(float duration, int timeout)
{
	ushort dat = 0;
	do
	{
		this->m_pDriver->doModbusIO(EK9000_SLAVE_ID, MODBUS_READ_HOLDING_REGISTERS, 0x1400, &dat, 1);
		epicsThreadSleep(duration);
		timeout--;
	} while ((dat | 0x200) == 0x200 && timeout > 0);

	if (timeout <= 0)
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

struct SMsg_t
{
	void* rec;
	void(*callback)(void*);
};

void CEK9000Device::QueueCallback(void(*callback)(void*), void* rec)
{
//	SMsg_t* msg = new SMsg_t;
//	this->queue->trySend(&msg, sizeof(callback));
}

void CEK9000Device::ExecuteCallbacks()
{
	this->Lock();
	SMsg_t* msg;
	while(this->queue->tryReceive(&msg, sizeof(msg)))
	{
		msg->callback(msg->rec);
	}
	this->Unlock();
}

const char *CEK9000Device::ErrorToString(int i)
{
	if(i == EK_EADSERR)
		return "ADS Error";
	else if(i == EK_EBADIP)
		return "Malformed IP address";
	else if(i == EK_EBADMUTEX)
		return "Invalid mutex";
	else if(i == EK_EBADPARAM)
		return "Invalid parameter";
	else if(i == EK_EBADPORT)
		return "Invalid port";
	else if(i == EK_EBADPTR)
		return "Invalid pointer";
	else if(i == EK_EBADTERM)
		return "Invalid terminal or slave";
	else if(i == EK_EBADTERMID)
		return "Invalid terminal id";
	else if(i == EK_EBADTYP)
		return "Invalid type";
	else if(i == EK_EERR)
		return "Unspecified error";
	else if(i == EK_EMODBUSERR)
		return "Modbus driver error";
	else if(i == EK_EMUTEXTIMEOUT)
		return "Mutex operation timeout";
	else if(i == EK_ENOCONN)
		return "No connection";
	else if(i == EK_ENODEV)
		return "Invalid device";
	else if(i == EK_ENOENT)
		return "No entry";
	else if(i == EK_EOK)
		return "No error";
	else if(i == EK_ETERMIDMIS)
		return "Terminal id mismatch";
	else if(i == EK_EWTCHDG)
		return "Watchdog error";
	return "Invalid error code";
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
		epicsPrintf("Invalid name passed.\n");
		return;
	}

	if (!ip)
	{
		epicsPrintf("Invalid IP passed.\n");
		return;
	}

	if (num < 0)
	{
		epicsPrintf("Invalid terminal count passed.\n");
		return;
	}

	if (port <= 0)
	{
		epicsPrintf("The port %i is invalid.\n", port);
		return;
	}

	CEK9000Device *dev;

	char ipbuf[64];
	sprintf(ipbuf, "%s:%i", ip, port);

	dev = CEK9000Device::Create(name, ipbuf, num);

	if (!dev)
	{
		epicsPrintf("Unable to create device: Unspecified error.\n");
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
		epicsPrintf("Invalid parameter passed!\n");
		return;
	}

	CEK9000Device *dev = g_pDeviceMgr->FindDevice(ek);

	/* If we cant find the device :( */
	if (!dev)
	{
		epicsPrintf("Unable to create terminal \"%s\": Device by the name of \"%s\" not found.\n",
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
		epicsPrintf("Unabel to create terminal %s: No terminal with the ID %s found.\n",
			name, type);
		return;
	}

	/* Check for out of boundries */
	if (id > dev->m_nTerms)
	{
		epicsPrintf("Unable to create terminal \"%s\": Terminal index out of range.\n", name);
		return;
	}

	int status = dev->AddTerminal(name, tid, id);
	
	if (status)
	{
		Error("ek9000ConfigureTerminal(): Failed to create terminal.");
		return;
	}
}

void ek9000Stat(const iocshArgBuf *args)
{
	const char *ek9k = args[0].sval;
	if (!ek9k || !g_pDeviceMgr)
	{
		epicsPrintf("Invalid parameter.\n");
		return;
	}
	CEK9000Device *dev = g_pDeviceMgr->FindDevice(ek9k);

	if (!dev)
	{
		epicsPrintf("Invalid device.\n");
		return;
	}

	if(dev->Lock())
	{
		Error("ek9000Stat(): %s\n", CEK9000Device::ErrorToString(EK_EMUTEXTIMEOUT));
		return;
	}

	bool connected = false;
	if (dev->VerifyConnection())
		connected = true;

	uint16_t ao = 0, ai = 0, bo = 0, bi = 0, tcp = 0, sn = 0, wtd = 0;
	uint16_t hver = 0, svermaj = 0, svermin = 0, sverpat = 0;
	uint16_t day = 0, month = 0, year = 0;

	dev->ReadProcessImageSize(ao, ai, bo, bi);
	dev->ReadNumTCPConnections(tcp);
	dev->ReadSerialNumber(sn);
	dev->ReadVersionInfo(hver, svermaj, svermin, sverpat);
	dev->ReadNumFallbacksTriggered(wtd);
	dev->ReadMfgDate(day, month, year);

	epicsPrintf("Device: %s\n", ek9k);
	if(connected)
		epicsPrintf("\tStatus: CONNECTED\n");
	else
		epicsPrintf("\tStatus: NOT CONNECTED\n");
	epicsPrintf("\tIP: %s\n", dev->m_pIP);
	epicsPrintf("\tAsyn Port Name: %s\n", dev->m_pPortName);
	epicsPrintf("\tAO size: %u [bytes]\n", ao);
	epicsPrintf("\tAI size: %u [bytes]\n", ai);
	epicsPrintf("\tBI size: %u [bits]\n", bi);
	epicsPrintf("\tBO size: %u [bits]\n", bo);
	epicsPrintf("\tTCP connections: %u\n", tcp);
	epicsPrintf("\tSerial number: %u\n", sn);
	epicsPrintf("\tHardware Version: %u\n", hver);
	epicsPrintf("\tSoftware Version: %u.%u.%u\n", svermaj, svermin, sverpat);
	epicsPrintf("\tFallbacks triggered: %u\n", wtd);
	epicsPrintf("\tMfg date: %u/%u/%u\n", month, day, year);

	for(int i = 0; i < dev->m_nTerms; i++)
	{
		if(!dev->m_pTerms[i].m_pRecordName)
			continue;
		epicsPrintf("\tSlave #%i:\n", i+1);
		epicsPrintf("\t\tType: %u\n", dev->m_pTerms[i].m_nTerminalID);
		epicsPrintf("\t\tRecord Name: %s\n", dev->m_pTerms[i].m_pRecordName);
		epicsPrintf("\t\tOutput Size: %u\n", dev->m_pTerms[i].m_nOutputSize);
		epicsPrintf("\t\tOutput Start: %u\n", dev->m_pTerms[i].m_nOutputStart);
		epicsPrintf("\t\tInput Size: %u\n", dev->m_pTerms[i].m_nInputSize);
		epicsPrintf("\t\tInput Start: %u\n", dev->m_pTerms[i].m_nInputStart);
	}

	dev->Unlock();
}

void ek9000EnableDebug(const iocshArgBuf* args)
{
	g_bDebug = true;
	epicsPrintf("Debug enabled.\n");
}

void ek9000DisableDebug(const iocshArgBuf* args)
{
	g_bDebug = false;
	epicsPrintf("Debug disabled.\n");
}

void ek9000List(const iocshArgBuf* args)
{
	for(CEK9000Device* dev = g_pDeviceMgr->FirstDevice(); dev; dev = g_pDeviceMgr->NextDevice())
	{
		epicsPrintf("Device: %s\n\tSlave Count: %i\n", dev->m_pName, dev->m_nTerms);
		epicsPrintf("\tIP: %s\n", dev->m_pIP);
		epicsPrintf("\tConnected: %s\n", dev->VerifyConnection() ? "TRUE" : "FALSE");
	}
}

void ek9000SetWatchdogTime(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	int time = args[1].ival;
	if(!ek9k)
		return;
	if(time < 0 || time > 60000)
		return;
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);
	if(!dev)
		return;
	dev->WriteWatchdogTime((uint16_t)time);
}

void ek9000SetWatchdogType(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	int type = args[1].ival;
	if(!ek9k)
		return;
	if(type < 0 || type > 2)
	{
		epicsPrintf("2 = disable watchdog\n");
		epicsPrintf("1 = enable on telegram\n");
		epicsPrintf("0 = enable on write\n");
		return;
	}
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);
	if(!dev) return;
	dev->WriteWatchdogType(type);
}

void ek9000SetPollTime(const iocshArgBuf* args)
{
	const char* ek9k = args[0].sval;
	int time = args[1].ival;
	if(!ek9k) return;
	if(time < 10 || time > 1000) return;
	CEK9000Device* dev = g_pDeviceMgr->FindDevice(ek9k);
	if(!dev) return;
	g_nPollDelay = time;
}

int ek9000RegisterFunctions()
{
	/* ek9000SetWatchdogTime(ek9k, time[int]) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"Time", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2};
		static const iocshFuncDef func = {"ek9000SetWatchdogTime", 2, args};
		iocshRegister(&func, ek9000SetWatchdogTime);
	}

	/* ek9000SetWatchdogType(ek9k, type[int]) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"Type", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2};
		static const iocshFuncDef func = {"ek9000SetWatchdogType", 2, args};
		iocshRegister(&func, ek9000SetWatchdogType);
	}
	
	/* ek9000SetPollTime(ek9k, type[int]) */
	{
		static const iocshArg arg1 = {"Name", iocshArgString};
		static const iocshArg arg2 = {"Type", iocshArgInt};
		static const iocshArg* const args[] = {&arg1, &arg2};
		static const iocshFuncDef func = {"ek9000SetPollTime", 2, args};
		iocshRegister(&func, ek9000SetPollTime);
	}
	
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

	/* ek9000Stat */
	{
		static const iocshArg arg1 = {"EK9000 Name", iocshArgString};
		static const iocshArg*const args[] = {&arg1};
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

	/* ek9000List */
	{
		static iocshFuncDef func = {"ek9000List", 0, NULL};
		iocshRegister(&func, ek9000List);
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
		epicsPrintf("Initializing EK9000 Couplers.\n");
		for (CEK9000Device *dev = g_pDeviceMgr->FirstDevice(); dev; dev = g_pDeviceMgr->NextDevice())
		{
			if (!dev->m_bInit)
				dev->InitTerminals();
		}
		epicsPrintf("Initialization Complete.\n");
		Utl_InitThread();
	}
	return 0;
}

static long ek9000_init_record(void *prec)
{
	epicsPrintf("FATAL ERROR: You should not use devEK9000 on any records!\n");
	epicsAssert(__FILE__, __LINE__, "FATAL ERROR: You should not use devEK9000 on any records!\n", "Jeremy L.");
	return 0;
}



//======================================================//
//
// Device support types for CoE configuration
// 		- Strings for CoE configuration should be like
//			this: @CoE ek9k,terminal,index,subindex
//
//======================================================//
// Common funcs/defs
struct ek9k_coe_param_t
{
	CEK9000Device* ek9k;
	CTerminal* pterm;
	int index;
	int subindex;
	enum {
		COE_TYPE_BOOL,
		COE_TYPE_INT8,
		COE_TYPE_INT16,
		COE_TYPE_INT32,
		COE_TYPE_INT64,
	} type;
};

struct ek9k_param_t
{
	CEK9000Device* ek9k;
	int reg;
	int len;
};

struct ek9k_conf_pvt_t
{
	ek9k_coe_param_t param;
};

int CoE_ParseString(const char* str, ek9k_coe_param_t* param);
int EK9K_ParseString(const char* str, ek9k_param_t* param);
//======================================================//


//-----------------------------------------------------------------//
// Configuration CoE RO parameter
static long ek9k_confli_init(int pass);
static long ek9k_confli_init_record(void* prec);
static long ek9k_confli_read_record(void* prec);

struct
{
	long		number;
	DEVSUPFUN	dev_report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	read_longin; /*returns: (-1,0)=>(failure,success)*/
}
devEK9KCoERO =
{
	5,
	NULL,
	(DEVSUPFUN)ek9k_confli_init,
	ek9k_confli_init_record,
	NULL,
	ek9k_confli_read_record,
};

epicsExportAddress(dset, devEK9KCoERO);

static long ek9k_confli_init(int pass)
{
	return 0;
}

static long ek9k_confli_init_record(void* prec)
{
	int64inRecord* precord = static_cast<int64inRecord*>(prec);
	ek9k_coe_param_t param;
	precord->dpvt = calloc(1, sizeof(ek9k_conf_pvt_t));
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);

	if(CoE_ParseString(precord->inp.value.instio.string, &param))
	{
		Error("ek9k_confli_init_record: Malformed input link string for record %s\n", 
			precord->name);
		return 1;
	}
	dpvt->param = param;
	return 0;
}

static long ek9k_confli_read_record(void* prec)
{
	int64inRecord* precord = static_cast<int64inRecord*>(prec);
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);

	if(!dpvt || !dpvt->param.ek9k) return -1;

	dpvt->param.ek9k->Lock();

	switch(dpvt->param.type)
	{
		case ek9k_coe_param_t::COE_TYPE_BOOL:
		{
			uint16_t buf;
			dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 1, &buf, dpvt->param.subindex);
			precord->val = static_cast<epicsInt64>(buf);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT8:
		{
			uint16_t buf;
			dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 1, &buf, dpvt->param.subindex);
			precord->val = static_cast<epicsInt64>(buf);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT16:
		{
			uint16_t buf;
			dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 1, &buf, dpvt->param.subindex);
			precord->val = static_cast<epicsInt64>(buf);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT32:
		{
			uint32_t buf;
			dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 2, reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex);
			precord->val = static_cast<epicsInt64>(buf);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT64:
		{
			uint64_t buf;
			dpvt->param.ek9k->doCoEIO(0, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 4, reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex);
			precord->val = static_cast<epicsInt64>(buf);
			break;
		}
		default: break;
	}

	dpvt->param.ek9k->Unlock();
	return 0;
}

//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// Configuration RW CoE parameter
static long ek9k_conflo_init(int pass);
static long ek9k_conflo_init_record(void* prec);
static long ek9k_conflo_write_record(void* prec);

struct
{
	long		number;
	DEVSUPFUN	dev_report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_longout;/*(-1,0)=>(failure,success*/
}
devEK9KCoERW =
{
	5,
	NULL,
	(DEVSUPFUN)ek9k_conflo_init,
	ek9k_conflo_init_record,
	NULL,
	ek9k_conflo_write_record,
};

epicsExportAddress(dset, devEK9KCoERW);

static long ek9k_conflo_init(int pass)
{
	return 0;
}

static long ek9k_conflo_init_record(void* prec)
{
	int64outRecord* precord = static_cast<int64outRecord*>(prec);
	ek9k_coe_param_t param;
	precord->dpvt = calloc(1, sizeof(ek9k_conf_pvt_t));
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);
	
	if(CoE_ParseString(precord->out.value.instio.string, &param))
	{
		Error("ek9k_conflo_init_record: Malformed input link string for record %s\n", 
			precord->name);
		return 1;
	}
	dpvt->param = param;
	return 0;
}

static long ek9k_conflo_write_record(void* prec)
{
	int64outRecord* precord = static_cast<int64outRecord*>(prec);
	ek9k_conf_pvt_t* dpvt = static_cast<ek9k_conf_pvt_t*>(precord->dpvt);
	int ret;

	if(!dpvt || !dpvt->param.ek9k) return -1;

	dpvt->param.ek9k->Lock();

	switch(dpvt->param.type)
	{
		case ek9k_coe_param_t::COE_TYPE_BOOL:
		{
			uint16_t buf = static_cast<epicsInt16>(precord->val);
			ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 1, &buf, dpvt->param.subindex, 1);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT8:
		{
			uint16_t buf = static_cast<epicsInt16>(precord->val);
			ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 1, &buf, dpvt->param.subindex, 1);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT16:
		{
			uint16_t buf = static_cast<epicsInt16>(precord->val);
			ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 1, &buf, dpvt->param.subindex, 2);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT32:
		{
			uint32_t buf = static_cast<epicsInt32>(precord->val);
			ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 2, reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex, 4);
			break;
		}
		case ek9k_coe_param_t::COE_TYPE_INT64:
		{
			uint64_t buf = precord->val;
			ret = dpvt->param.ek9k->doCoEIO(1, dpvt->param.pterm->m_nTerminalIndex,
				dpvt->param.index, 4, reinterpret_cast<uint16_t*>(&buf), dpvt->param.subindex, 8);
			break;
		}
		default: break;
	}

	dpvt->param.ek9k->Unlock();
	return 0;
}

//-----------------------------------------------------------------//

int CoE_ParseString(const char* str, ek9k_coe_param_t* param)
{
	int strl = strlen(str);
	CEK9000Device* pcoupler = 0;
	CTerminal* pterm = 0;
	int termid;
	int i, bufcnt = 0;
	char buf[512]; 
	char* buffers[5];

	if(str[0] == 0) return 1;

	memset(buf, 0, 512);
	strcpy(buf, str);

	/* Tokenize & separate into substrings */
	for(char* tok = strtok((char*)str, ","); tok; tok = strtok(NULL, ","))
	{
		buffers[bufcnt] = tok;
		++bufcnt;
	}
	for(i = 0; i < strl; i++)
		if(buf[i] == ',') buf[i] = 0;

	/* Finally actually parse the integers, find the ek9k, etc. */
	for(CEK9000Device* dev = g_pDeviceMgr->FirstDevice(); dev; dev = g_pDeviceMgr->NextDevice())
	{
		if(strncmp(dev->m_pName, buffers[0], strlen(dev->m_pName)) == 0)
		{
			pcoupler = dev;
			break;
		}
	}

	if(!pcoupler)
	{
		Error("Coupler not found.\n");
		return 1;
	}

	if(bufcnt < 4) return 1;

	/* Determine the CoE type */
	if(strncmpnc(buffers[4], "bool", 4) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_BOOL;
	else if(strncmpnc(buffers[4], "int16", 5) == 0 || strncmpnc(buffers[4], "uint16", 6) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT16;
	else if(strncmpnc(buffers[4], "int32", 5) == 0 || strncmpnc(buffers[4], "uint32", 6) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT32;
	else if(strncmpnc(buffers[4], "int64", 5) == 0 || strncmpnc(buffers[4], "uint64", 6) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT64;
	else if(strncmpnc(buffers[4], "int8", 4) == 0 || strncmpnc(buffers[4], "uint8", 5) == 0)
		param->type = ek9k_coe_param_t::COE_TYPE_INT8;
	else
		return 1;

	termid = atoi(buffers[1]);
	if(errno != 0) return 1;
	pterm = &pcoupler->m_pTerms[termid-1];

	param->pterm = pterm;
	param->ek9k = pcoupler;
	param->index = strtol(buffers[2], 0, 16);
	if(errno != 0) return 1;

	param->subindex = strtol(buffers[3], 0, 16);
	if(errno != 0) return 1;

	return 0;
}

//-----------------------------------------------------------------//
// EK9K RO configuration/status
static long ek9k_ro_init(int pass);
static long ek9k_ro_init_record(void* prec);
static long ek9k_ro_read_record(void* pprec);

struct
{
	long		number;
	DEVSUPFUN	dev_report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_longout;/*(-1,0)=>(failure,success*/
}
devEK9000ConfigRO =
{
	5,
	NULL,
	(DEVSUPFUN)ek9k_ro_init,
	ek9k_ro_init_record,
	NULL,
	ek9k_ro_read_record,
};

epicsExportAddress(dset, devEK9000ConfigRO);

static long ek9k_ro_init(int pass)
{
	return 0;
}

static long ek9k_ro_init_record(void* prec)
{
	longinRecord* precord = static_cast<longinRecord*>(prec);
	precord->dpvt = calloc(1, sizeof(ek9k_param_t));
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	ek9k_param_t param;

	/* parse the string to obtain various info */
	if(EK9K_ParseString(precord->inp.value.instio.string, &param))
	{
		Error("Maloformed modbus string in record %s\n", precord->name);
		return 1;
	}
	*dpvt = param;
	return 0;
}

static long ek9k_ro_read_record(void* prec)
{
	longinRecord* precord = static_cast<longinRecord*>(prec);
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	CEK9000Device* dev = dpvt->ek9k;
	uint16_t buf;

	if(!dev) return -1;

	dev->Lock();

	dev->doEK9000IO(0, dpvt->reg, 1, &buf);
	precord->val = buf;

	dev->Unlock();
	return 0;
}

//-----------------------------------------------------------------//

//-----------------------------------------------------------------//
// EK9K RW configuration
static long ek9k_rw_init(int pass);
static long ek9k_rw_init_record(void* prec);
static long ek9k_rw_write_record(void* pprec);

struct
{
	long		number;
	DEVSUPFUN	dev_report;
	DEVSUPFUN	init;
	DEVSUPFUN	init_record; /*returns: (-1,0)=>(failure,success)*/
	DEVSUPFUN	get_ioint_info;
	DEVSUPFUN	write_longout;/*(-1,0)=>(failure,success*/
}
devEK9000ConfigRW =
{
	5,
	NULL,
	(DEVSUPFUN)ek9k_rw_init,
	ek9k_rw_init_record,
	NULL,
	ek9k_rw_write_record,
};

epicsExportAddress(dset, devEK9000ConfigRW);

static long ek9k_rw_init(int pass)
{
	return 0;
}

static long ek9k_rw_init_record(void* prec)
{
	longoutRecord* precord = static_cast<longoutRecord*>(prec);
	precord->dpvt = calloc(1, sizeof(ek9k_param_t));
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	ek9k_param_t param;

	/* parse the string to obtain various info */
	if(EK9K_ParseString(precord->out.value.instio.string, &param))
	{
		Error("Maloformed modbus string in record %s\n", precord->name);
		return 1;
	}
	*dpvt = param;
	return 0;
}

static long ek9k_rw_write_record(void* prec)
{
	longinRecord* precord = static_cast<longinRecord*>(prec);
	ek9k_param_t* dpvt = static_cast<ek9k_param_t*>(precord->dpvt);
	CEK9000Device* dev = dpvt->ek9k;
	uint16_t buf = precord->val;

	if(!dev) return -1;

	dev->Lock();

	dev->doEK9000IO(1, dpvt->reg, 1, &buf);

	dev->Unlock();
	return 0;
}

//-----------------------------------------------------------------//

/* The string will be in the format EK9K,0x1002 */
int EK9K_ParseString(const char* str, ek9k_param_t* param)
{
	char buf[512];
	const char *pek9k = 0, *preg = 0;

	memset(buf, 0, 512);
	strncpy(buf, str, 512);

	if(!buf[0]) return 1;
	pek9k = buf;

	/* Read until we hit a comma, then replace */
	int strl = strlen(buf);
	for(int i = 0; i < strl; i++)
	{
		if(buf[i] == ',')
		{
			buf[i] = '\0';
			preg = &buf[i+1];
			break;
		}
	}
	if(!preg) return 1;

	/* Find the coupler */
	param->ek9k = g_pDeviceMgr->FindDevice(pek9k);
	param->reg = strtol(preg, 0, 16);

	if(!param->ek9k)
	{
		printf("Unable to find the specified coupler\n");
		return 1;
	}
	return 0;
}
