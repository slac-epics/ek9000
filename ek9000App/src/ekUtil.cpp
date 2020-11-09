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

using namespace util;

/* Maintain a list of handles to iocsh args */
/* This really wont be needed, but it helps us pretend that we aren't leaking memory */
struct iocshHandles_t
{
	iocshArg *args;
	iocshArg **pargs;
	iocshFuncDef func;
};
std::vector<iocshHandles_t *> functions;

/* Register iocsh functions */
void util::iocshRegister(const char *name, void (*pfn)(const iocshArgBuf *), std::initializer_list<iocshArg> args)
{
	iocshHandles_t *handle = static_cast<iocshHandles_t *>(malloc(sizeof(iocshHandles_t)));
	handle->args = static_cast<iocshArg *>(malloc(args.size() * sizeof(iocshArg)));
	handle->pargs = static_cast<iocshArg **>(malloc(args.size() * sizeof(iocshArg *)));
	auto it = args.begin();
	for (int i = 0; i < args.size() && it; i++, it++)
		handle->args[i] = *it;
	for (int i = 0; i < args.size(); i++)
		handle->pargs[i] = &handle->args[i];
	handle->func = {name, (int)args.size(), handle->pargs};
	::iocshRegister(&handle->func, pfn);
	functions.push_back(handle);
}

/*
 * @ek9k_name,slave_num,channel
 */
void *util::parseAndCreateDpvt(char *instio)
{
	if (!instio)
		return nullptr;
	int pindex = 0;

	for (char *subst = strtok(instio, ","); subst; subst = strtok(NULL, ","), pindex++)
	{
	}

	if (pindex < 2)
	{
		epicsPrintf("Syntax error in instio string: %s\n", instio);
		return nullptr;
	}

	int ncommas = 0;
	size_t len = strlen(instio);
	for (int i = 0; i < len; i++)
	{
	}
}

const STerminalInfoConst_t *util::FindTerminal(int id)
{
	for (int i = 0; i < NUM_TERMINALS; i++)
		if (g_pTerminalInfos[i]->m_nID == id)
			return g_pTerminalInfos[i];
	return nullptr;
}

void util::Log(const char *fmt, ...)
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

void util::Warn(const char *fmt, ...)
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

void util::Error(const char *fmt, ...)
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

long util::setupCallback(void *rec, void (*pCallback)(CALLBACK *))
{
	CALLBACK *callback = (CALLBACK *)calloc(1, sizeof(CALLBACK));
	*callback = *(CALLBACK *)pCallback;
	callbackSetCallback(pCallback, callback);
	callbackSetUser(rec, callback);
	callbackSetPriority(priorityHigh, callback);
	callbackRequest(callback);
	return 0;
}

terminal_info_t util::getTerminalInfo(uint32_t id)
{
}

terminal_info_t util::getTerminalInfo(const char *id)
{
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

bool util::ParseLinkSpecification(const char* link, ELinkType linkType, LinkSpecification_t outSpec)
{
	if(!link) return false;

	switch(linkType) {
		case util::LINK_INST_IO:
		{
			int linkLen = strlen(link);
			if(linkLen <= 1) return false;
			if(link[0] != '@') return false;

			/* Count the number of commas, which correspond to params */
			int paramCount = 0;
			for(int i = 0; i < linkLen; i++)
				if(link[i] == ',') paramCount++;
			LinkParameter_t* linkParams = nullptr;

			/* Nothing to parse? */
			if(paramCount == 0)
				return true;


			/* Tokenize the string */
			link++;
			char buf[1024];
			snprintf(buf, sizeof(buf), "%s", link);
			char* param, *value;
			int paramIndex = 0;
			param = value = nullptr;
			for(char* tok = strtok(buf, ","); tok; tok = strtok(NULL, ","))
			{
				param = tok;
				/* Search for the = to break the thing up */
				for(int i = 0; tok[i]; i++)
				{
					if(tok[i] == '=')
						value = &tok[i+1];
				}
				/* If NULL, it's the end of the string and the param is malformed */
				if(*value == 0) {
					if(linkParams) free(linkParams);
					return false;
				}

				/* Probably should just use stack allocation here (alloca), but that's not really portable (technically is, but still) to non-POSIX platforms (e.g. windows) */
				if(!linkParams)
					linkParams = (LinkParameter_t*)malloc(sizeof(LinkParameter_t) * paramCount);
				
				/* Add new param to the list */
				outSpec.params[paramIndex].key = epicsStrDup(param);
				outSpec.params[paramIndex].value = epicsStrDup(value);
				paramIndex++;
			}

			outSpec.numParams = paramCount;
			outSpec.params = linkParams;
			
			return true;
		}
		default:
			break;
	}

	return false;
}

void util::DestroyLinkSpecification(LinkSpecification_t& spec)
{
	for(int i = 0; spec.params && i < spec.numParams; i++)
	{
		free(spec.params[i].key);
		free(spec.params[i].value);
	}
	if(spec.params)
		free(spec.params);
	spec.params = nullptr;
	spec.numParams = 0;
}
