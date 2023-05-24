/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
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

#include "terminal.h"

// C++11 interop - mainly utils for catching problems in code
#if __cplusplus >= 201103L
#define CONSTEXPR constexpr
#define OVERRIDE override
#define FINAL final
#define MAYBE_UNUSED [[maybe_unused]]
#else
#define CONSTEXPR static const
#define OVERRIDE
#define FINAL
#define MAYBE_UNUSED
#endif

#undef UNUSED
#define UNUSED(x) (void)x

// This comes from a windows header
#undef CALLBACK

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

// The following macros are for validating terminal_types.g.h against any PDO structs defined in code

#define DEFINE_SINGLE_CHANNEL_INPUT_PDO(_pdoTypeName, _terminalType)                                                   \
	void _terminalType##_t ::_pdo_input_check() {                                                                      \
		STATIC_ASSERT(sizeof(_pdoTypeName) == (_terminalType##_t::INPUT_SIZE / _terminalType##_t::NUM_INPUTS) * 2);    \
	}

#define DEFINE_SINGLE_CHANNEL_OUTPUT_PDO(_pdoTypeName, _terminalType)                                                  \
	void _terminalType##_t ::_pdo_output_check() {                                                                     \
		STATIC_ASSERT(sizeof(_pdoTypeName) == (_terminalType##_t::OUTPUT_SIZE / _terminalType##_t::NUM_OUTPUTS) * 2);  \
	}

#define DEFINE_DUMMY_INPUT_PDO_CHECK(_terminalType) void _terminalType##_t ::_pdo_input_check(){};

#define DEFINE_DUMMY_OUTPUT_PDO_CHECK(_terminalType) void _terminalType##_t ::_pdo_output_check(){};

/**
 * Determine size of an array
 */
template <class T, size_t N> size_t ArraySize(T (&arr)[N]) {
	UNUSED(arr);
	return N;
}

namespace util
{

#if __cplusplus >= 201103L
using std::is_same;
#else
template <class T, class U> struct is_same { static bool value; };

template <class T, class U> bool is_same<T, U>::value = false;

template <class T> struct is_same<T, T> { static bool value; };

template <class T> bool is_same<T, T>::value = true;
#endif

template <class T> bool DpvtValid(void* dpvt) {
	T* pdpvt = static_cast<T*>(dpvt);
	if (!dpvt || !pdpvt->device)
		return false;
	return true;
}

/**
 * Look up a terminal by ID and return a structure containing info about it
 */
const terminal_t* FindTerminal(unsigned int id);

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
