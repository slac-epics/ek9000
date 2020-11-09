/*
 * util.h
 *
 * Common utilities for use with EPICS
 *
 */ 
#pragma once

#include <initializer_list>
#include <iocsh.h>

/* Record includes */
#include <boRecord.h>
#include <biRecord.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <epicsSpin.h>
#include <epicsMutex.h>
#include <epicsAtomic.h>

/* STL includes */
#include <string>
#include <list>

/* List of all terminals */
#include "terminals.h"

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
	class devEK9000* pdrv;
	int slave, terminal, channel;
	int baseaddr, len;
	LinkSpecification_t linkSpec;
	char* terminalType;
	char* mapping;
	char* representation;
} terminal_dpvt_t;

typedef struct 
{
	const char* m_name;
	uint32_t m_id;
	uint16_t m_outputSize;
	uint16_t m_inputSize;
} terminal_info_t;

template<class MutexT>
class CAutoLockWrapper
{
	MutexT* m_mutex;
public:
	explicit CAutoLockWrapper(MutexT* mutex) :
		m_mutex(mutex)
	{
		m_mutex->lock();
	}

	~CAutoLockWrapper()
	{
		m_mutex->unlock();
	}
};

#define AUTO_LOCK(x) CAutoLockWrapper<epicsMutex> __auto_lock(x)

namespace util
{
	void iocshRegister(const char* name, void(*pfn)(const iocshArgBuf*), std::initializer_list<iocshArg> args);
	
	/*
	 * Parses and creates a device private structure for the terminal
	 * instio is your instio string passed into the record's INP field.
	 */ 
	void* parseAndCreateDpvt(char* instio);

	/**
	 * Look up a terminal by ID and return a structure containing info about it
	 */ 
	const STerminalInfoConst_t* FindTerminal(int id);

	/**
	 * Logging routines
	 */ 
	void Log(const char* fmt, ...);
	void Warn(const char* fmt, ...);
	void Error(const char* fmt, ...);

	/**
	 * Call this to setup a callback. Can be used in-place of the read or write functions in the dpvt struct
	 */ 
        long setupCallback(void* rec, void(*pCallback)(CALLBACK*));

	terminal_info_t getTerminalInfo(uint32_t id);
	terminal_info_t getTerminalInfo(const char* id);

	enum ELinkType 
	{
		BAD = 0,
		LINK_INST_IO,
	};

	bool ParseLinkSpecification(const char* link, ELinkType linkType, LinkSpecification_t& spec);

	void DestroyLinkSpecification(LinkSpecification_t& spec);

	/* Performs common dpvt setup */
	template<class RecordT>
	void setupCommonDpvt(RecordT* prec, terminal_dpvt_t& dpvt);

	terminal_dpvt_t emptyDpvt();

	template<class RecordT>
	void destroyDpvt(RecordT* prec, terminal_dpvt_t& dpvt);
}

