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

#include "terminal_types.g.h"

using namespace util;

/* Maintain a list of handles to iocsh args */
/* This really wont be needed, but it helps us pretend that we aren't leaking memory */
struct iocshHandles_t {
	iocshArg* args;
	iocshArg** pargs;
	iocshFuncDef func;
};
std::vector<iocshHandles_t*> functions;

const terminal_t* util::FindTerminal(unsigned int id) {
	for (unsigned int i = 0; i < ArraySize(s_terminalInfos); i++)
		if (s_terminalInfos[i].id == id)
			return &s_terminalInfos[i];
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

/**
 * We also handle some backwards compatibility here.
 */
bool util::setupCommonDpvt(const char* recName, const char* inp, TerminalDpvt_t& dpvt) {
	static const char* function = "util::setupCommonDpvt<RecordT>()";

	if (!inp || !util::ParseLinkSpecification(inp, INST_IO, dpvt.linkSpec)) {
		// This is likely a legacy setup, fallback to old logic
		devEK9000Terminal* term = NULL;
		if (!(term = devEK9000Terminal::ProcessRecordName(recName, &dpvt.channel)))
			return false;
		dpvt.pdrv = term->m_device;
		dpvt.pterm = term;
		dpvt.terminalType = term->m_terminalId;
		return true;
	}

	/* Parse the params passed via INST_IO stuff */
	const int paramCount = dpvt.linkSpec.size();
	for (int i = 0; i < paramCount; ++i) {
		std::pair<std::string, std::string>& param = dpvt.linkSpec.at(i);

		/* Device name */
		if (strcmp(param.first.c_str(), "device") == 0) {
			bool found = false;
			std::list<devEK9000*>& devList = GlobalDeviceList();
			for (std::list<devEK9000*>::iterator x = devList.begin(); x != devList.end(); ++x) {
				// for (const auto& x : GlobalDeviceList()) {
				if (strcmp((*x)->m_name.data(), param.second.c_str()) == 0) {
					dpvt.pdrv = *x;
					found = true;
					break;
				}
			}
			if (!found) {
				epicsPrintf("%s (when parsing %s): invalid device name: %s\n", function, recName, param.second.c_str());
				return false;
			}
		}
		/* Terminal position in rail (1=first) */
		else if (strcmp(param.first.c_str(), "pos") == 0) {
			int term = atoi(param.second.c_str());
			/* Max supported devices by the EK9K is 255 */
			if (term < 0 || term > 255) {
				epicsPrintf("%s (when parsing %s): invalid rail position: %i\n", function, recName, term);
				return false;
			}
			dpvt.pos = term;
		}
		/* Channel number */
		else if (strcmp(param.first.c_str(), "channel") == 0) {
			int channel = atoi(param.second.c_str());
			/* No real max here, but I think it's good to limit this to 8k as nothing has this many channels */
			if (channel < 0 || channel > 8192) {
				epicsPrintf("%s (when parsing %s): invalid channel: %i\n", function, recName, channel);
				return false;
			}
			dpvt.channel = channel;
		}
		/* Terminal type string e.g. EL3064 */
		else if (strcmp(param.first.c_str(), "type") == 0) {
			const char* tid = param.second.c_str();
			if (!strncmp(tid, "EL", 2))
				tid += 2;
			if (epicsParseInt32(tid, &dpvt.terminalType, 10, NULL) != 0) {
				epicsPrintf("%s (when parsing %s): unable to parse terminal ID from string '%s'\n", function, recName, param.second.c_str());
				return false;
			}
		}
		else {
			epicsPrintf("%s (when parsing %s): ignored unknown param %s\n", function, recName, param.first.c_str());
		}
	}
	
	if (!dpvt.pdrv) {
		epicsPrintf("%s (when parsing %s): no device specified\n", function, recName);
		memset(&dpvt, 0, sizeof(dpvt));
		return false;
	}

	// TODO: It is likely that we'll need to recompute the coupler's mapping in here if we ever add
	//  support for alternative PDO mapping types that affect PDO mapping on the device.
	
	/* Resolve terminal */
	dpvt.pterm = dpvt.pdrv->TerminalByIndex(dpvt.pos);
	dpvt.pterm->SetRecordName(recName);
	dpvt.pterm->Init(dpvt.terminalType, dpvt.pos);
	if (dpvt.pterm == NULL) {
		epicsPrintf("%s (when parsing %s): unable to find terminal\n", function, recName);
		memset(&dpvt, 0, sizeof(dpvt));
		return false;
	}
	
	return true;
}


/**
 * Right now only the INST_IO link type is supported.
 * INST_IO links cannot have any spaces in them, so @1,2,3,5 is valid
 *
 * To keep backwards compatibility, parameters will be named and are not ordered in any particilar way
 * Example of our instio syntax:
 * @Something=A,OtherParam=B,Thing=C
 *
 */

bool util::ParseLinkSpecification(const char* link, int linkType, LinkSpec_t& outSpec) {
	if (!link || link[0] == '\0')
		return false;

	switch (linkType) {
		case INST_IO:
			{
				char buf[2048];
				strncpy(buf, link, sizeof(buf));
				buf[sizeof(buf)-1] = 0;
				
				// Tokenize by commas
				for (char* tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
					// Split based on equals
					char* s = strpbrk(tok, "=");
					if (!s) {
						outSpec.clear();
						return false;
					}
					*s = 0; s++;
					outSpec.push_back(std::make_pair(std::string(tok), std::string(s)));
				}
				
				return true;
			}
		default:
			break;
	}

	return false;
}
