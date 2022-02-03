/*
 * util.cpp
 *
 * Common utilities for use with epics
 *
 */
#include "ekUtil.h"
#include <memory.h>
#include <stdlib.h>
#include <vector>
#include <epicsPrint.h>
#include <epicsStdio.h>
#include <epicsTime.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsString.h>

#include "devEK9000.h"

using namespace util;

/* Maintain a list of handles to iocsh args */
/* This really wont be needed, but it helps us pretend that we aren't leaking memory */
struct iocshHandles_t {
	iocshArg* args;
	iocshArg** pargs;
	iocshFuncDef func;
};
std::vector<iocshHandles_t*> functions;

const STerminalInfoConst_t* util::FindTerminal(unsigned int id) {
	for (unsigned int i = 0; i < ArraySize(g_pTerminalInfos); i++)
		if (g_pTerminalInfos[i]->m_nID == id)
			return g_pTerminalInfos[i];
	return NULL;
}

void util::Log(const char* fmt, ...) {
	time_t clk = time(0);
	tm _tm;

	epicsTime_localtime(&clk, &_tm);
	epicsPrintf("%i:%i ", _tm.tm_hour, _tm.tm_min);

	va_list list;
	va_start(list, fmt);
	epicsVprintf(fmt, list);
	va_end(list);
}

void util::Warn(const char* fmt, ...) {
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

void util::Error(const char* fmt, ...) {
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

long util::setupCallback(void* rec, void (*pCallback)(CALLBACK*)) {
	CALLBACK* callback = (CALLBACK*)calloc(1, sizeof(CALLBACK));
	*callback = *(CALLBACK*)pCallback;
	callbackSetCallback(pCallback, callback);
	callbackSetUser(rec, callback);
	callbackSetPriority(priorityHigh, callback);
	callbackRequest(callback);
	return 0;
}
