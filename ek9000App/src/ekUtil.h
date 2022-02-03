/*
 * util.h
 *
 * Common utilities for use with EPICS
 *
 */
#pragma once

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

#include "ekLocal.h"

#include "terminals.h"
#include "devEK9000.h"

typedef struct {
	const char* m_name;
	uint32_t m_id;
	uint16_t m_outputSize;
	uint16_t m_inputSize;
} terminal_info_t;

template <class MutexT> class CAutoLockWrapper {
	MutexT* m_mutex;

public:
	explicit CAutoLockWrapper(MutexT* mutex) : m_mutex(mutex) {
		m_mutex->lock();
	}

	~CAutoLockWrapper() {
		m_mutex->unlock();
	}
};

#define AUTO_LOCK(x) CAutoLockWrapper<epicsMutex> __auto_lock(x)

/**
 * Determine size of an array
 */
template<class T, size_t N>
size_t ArraySize(T(&arr)[N]) {
	return N;
}

namespace util
{
/**
 * Look up a terminal by ID and return a structure containing info about it
 */
const STerminalInfoConst_t* FindTerminal(unsigned int id);

/**
 * Logging routines
 */
void Log(const char* fmt, ...);
void Warn(const char* fmt, ...);
void Error(const char* fmt, ...);

/**
 * Call this to setup a callback. Can be used in-place of the read or write functions in the dpvt struct
 */
long setupCallback(void* rec, void (*pCallback)(CALLBACK*));

} // namespace util
