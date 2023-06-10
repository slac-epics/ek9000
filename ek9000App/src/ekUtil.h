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
#include <mbboDirectRecord.h>

/* STL includes */
#include <string>
#include <list>
#include <vector>
#include <cstring>
#include <type_traits>

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

typedef std::vector<std::pair<std::string, std::string>> LinkSpec_t;

struct TerminalDpvt_t {
	class devEK9000* pdrv;			// Pointer to the coupler itself
	int pos;						// Position in the rail (first=1)
	class devEK9000Terminal* pterm;	// Pointer to the terminal, which contains mappings
	int channel;					// Channel number within the terminal
	LinkSpec_t linkSpec;			// All link parameters
	int terminalType;				// Terminal type ID (i.e. 3064 from EL3064)
};


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

#if __cplusplus >= 202002L

namespace detail {

template<typename T>
concept BASE_RECORD = requires(T a) { { a.name } -> std::convertible_to<const char*>; };

template<typename T>
concept OUTPUT_RECORD = requires(T a) { { a.out } -> std::same_as<link&>; };

template<typename T>
concept INPUT_RECORD = requires(T a) { { a.inp } -> std::same_as<link&>; };

}

template<typename T>
concept RECORD_TYPE = detail::BASE_RECORD<T> && (detail::INPUT_RECORD<T> || detail::OUTPUT_RECORD<T>);

#else

#define RECORD_TYPE typename

#endif

inline bool DpvtValid(TerminalDpvt_t* dpvt) {
	return dpvt && dpvt->pdrv;
}


/**
 * @brief Parse a INP string
 * @param link string from INP PV
 * @param linkType Type of link. INST_IO is the only supported type right now.
 * @param outSpec
 * @returns true if the link was parsed successfully
 */
bool ParseLinkSpecification(const char* link, int linkType, LinkSpec_t& outSpec);

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

inline TerminalDpvt_t* allocDpvt() { return (TerminalDpvt_t*)calloc(1,sizeof(TerminalDpvt_t)); }

bool setupCommonDpvt(const char* recName, const char* inp, TerminalDpvt_t& dpvt);

/** The below template code is kinda ugly. I'd like to use if constexpr or concepts for overload resolution (so we don't need to specialize for all output records), 
  * but we're bound to C++03 unfortunately! */

/**
 * @brief Setup device private info
 * @tparam RecordT Record type (e.g. aiRecord)
 * @param prec Pointer to the record
 * @param dpvt Where we put the dpvt
 * @returns true if success
 */
template <RECORD_TYPE RecordT>
inline bool setupCommonDpvt(RecordT* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->inp.text, dpvt);
}

template<>
inline bool setupCommonDpvt<boRecord>(boRecord* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->out.text, dpvt);
}
template<>
inline bool setupCommonDpvt<mbboDirectRecord>(mbboDirectRecord* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->out.text, dpvt);
}
template<>
inline bool setupCommonDpvt<aoRecord>(aoRecord* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->out.text, dpvt);
}


} // namespace util

// Clear pre-C++20 concept hacks
#undef RECORD_TYPE
