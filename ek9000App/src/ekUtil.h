/*
 * util.h
 *
 * Common utilities for use with EPICS
 *
 */ 
#pragma once

#include <initializer_list>
#include <iocsh.h>

#include <epicsAtomic.h>

/* List of all terminals */
#include "terminals.h"

typedef struct 
{
	class drvEK9000* pdrv;
	int slave, terminal, channel;
	int baseaddr, len;
} terminal_dpvt_t;

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
}

