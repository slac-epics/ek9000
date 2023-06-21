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
#include <asynDriver.h>
#include <compilerSpecific.h>

/* STL includes */
#include <string>
#include <list>
#include <vector>
#include <cstring>

#if __cplusplus >= 201103L
#include <type_traits>
#endif

#include "terminal.h"

// C++11 interop - these are utils for catching problems in code. Anything from C++11 or newer that affects compilation (e.g. concepts/constraints for template overload resolution) 
// are forbidden in this codebase. Hopefully one day we'll be able to upgrade to C++11, or even C++14!
#if __cplusplus >= 201103L
#define CONSTEXPR constexpr
#define OVERRIDE override
#define FINAL final
#define MAYBE_UNUSED [[maybe_unused]]
#define DELETE_CTOR(x) x = delete		/* Sucks! Mainly to prevent misuse of classes, if you need a deleted ctor for other reasons, just make it private. */
#else
#define CONSTEXPR static const
#define OVERRIDE
#define FINAL
#define MAYBE_UNUSED
#define DELETE_CTOR(x)
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

typedef std::vector< std::pair<std::string, std::string> > LinkSpec_t;

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

namespace util {

/**
 * Simple format string 
 * TODO: When we upgrade to C++11, make this templated with a constant for string length
 */
class FmtStr {
public:
	DELETE_CTOR(FmtStr());
	FmtStr(const char* fmt, ...) EPICS_PRINTF_STYLE(2,3) {
		va_list va;
		va_start(va, fmt);
		vsnprintf(str_, sizeof(str_), fmt, va);
		va_end(va);
	}

	const char* c_str() const { return str_; }
	operator const char*() const { return str_; }

private:
	char str_[2048];
};

}

/**
 * Logging helpers
 */
#define _LOG_ASYN(_traceType, _device, ...) do { if(_device) \
		asynPrint(_device->GetAsynUser(), _traceType, "%s: %s", __FUNCTION__, util::FmtStr(__VA_ARGS__).c_str()); \
	else \
		epicsPrintf("%s: %s", __FUNCTION__, util::FmtStr(__VA_ARGS__).c_str()); } while(0)

#define LOG_ERROR(_device, ...) _LOG_ASYN(ASYN_TRACE_ERROR, _device, __VA_ARGS__)
#define LOG_WARNING(_device, ...) _LOG_ASYN(ASYN_TRACE_WARNING, _device, __VA_ARGS__)
#define LOG_INFO(_device, ...) _LOG_ASYN(ASYN_TRACE_FLOW, _device, __VA_ARGS__)

namespace util
{

/**
 * C++03 doesn't have is_same, so implement that ourselves
 */
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
	return setupCommonDpvt(prec->name, prec->inp.value.instio.string, dpvt);
}

template<>
inline bool setupCommonDpvt<boRecord>(boRecord* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->out.value.instio.string, dpvt);
}
template<>
inline bool setupCommonDpvt<mbboDirectRecord>(mbboDirectRecord* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->out.value.instio.string, dpvt);
}
template<>
inline bool setupCommonDpvt<aoRecord>(aoRecord* prec, TerminalDpvt_t& dpvt) {
	return setupCommonDpvt(prec->name, prec->out.value.instio.string, dpvt);
}


} // namespace util

// Clear pre-C++20 concept hacks
#undef RECORD_TYPE
