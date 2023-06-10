
#include "epicsTest.h"
#include "errlog.h"
#include <epicsStdlib.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <string.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#ifdef __linux__
#include <unistd.h>
#endif

namespace chrono = std::chrono;

static int s_testExitOnError = 0;

#define COLOR_PRINTF(color, fmt, ...) do { setStdoutColor(color); printf(fmt, __VA_ARGS__); setStdoutColor(ANSI_ESC_RESET); } while(0)

static void setStdoutColor(const char* color);
static bool supportsANSIColor();

inline bool startswith(const char* str, const char* prefix) {
	return !strncmp(str, prefix, strlen(prefix));
}


static std::vector<UnitTest*>& registeredTests() {
	static std::vector<UnitTest*> s;
	return s;
}

UnitTest::UnitTest(const char* test, const char* testSuite, testProc testProc) :
	proc_(testProc), name_(test), desc_(testSuite) {
	registeredTests().push_back(this);
}

bool UnitTest::testAssert(bool cond, const std::string& failMsg, const char* file, int line, bool fatal) {
	asserts_++;
	if (!cond) {
		failed_++;
		
		COLOR_PRINTF(ANSI_ESC_YELLOW, "\n  %s FAILED: ", fatal ? "ASSERTION" : "EXPECTATION");
		
		puts(failMsg.c_str());
		printf("    in %s on line %d\n", file, line);
		
		return !fatal; // Abort the test if this is considered fatal
	}
	return true;
}

std::string UnitTest::fullName() const {
	return desc_ + "." + name_;
}

bool UnitTest::exec() {
	this->proc_(this);
	return failed_ == 0;
}

void UnitTest::report() {
	
}

void UnitTest::reset() {
	this->failed_ = this->asserts_ = 0;
}

int runTestsWithFilters(const std::vector<std::string>& filters) {
	auto& tests = registeredTests();
	
	using clock = chrono::system_clock;
	auto start = clock::now();
	
	int failures = 0, executed = 0;
	for(auto& test : tests) {
		// Apply filters if any requested
		auto testName = test->fullName();
		if (!filters.empty()) {
			if (std::none_of(filters.begin(), filters.end(), [testName](const std::string& filter) -> bool {
				return epicsStrGlobMatch(testName.c_str(), filter.c_str());
			})) {
				continue;
			}
		}

		COLOR_PRINTF(ANSI_ESC_CYAN, "Running test %s...", testName.c_str());
		
		// Reset before exec so we have a clean slate
		test->reset();
		
		// Run the test!
		if (!test->exec()) {
			COLOR_PRINTF(ANSI_ESC_RED, "FAILED (%u/%u)\n", test->passedAsserts(), test->asserts());
			++failures;
		}
		else {
			COLOR_PRINTF(ANSI_ESC_GREEN, "PASSED (%u/%u)\n", test->passedAsserts(), test->asserts());
		}
		++executed;
	}
	
	auto duration = chrono::duration_cast<chrono::milliseconds>(clock::now() - start).count();
	
	// Generate a quick report
	auto passed = executed - failures;
	float ratio = executed > 0 ? (passed / float(executed) * 100.f) : 100.f;
	COLOR_PRINTF(ANSI_ESC_GREEN, "\n%d/%d (%.1f%%) PASSED in %.2f seconds\n", passed, executed, ratio, duration / 1000.f);
	if (failures > 0) {
		COLOR_PRINTF(ANSI_ESC_RED, "%d tests FAILED!\n", failures );
	}
	return failures != 0;
}

int unit::runTests(int argc, char** argv) {
	std::vector<std::string> filters;
	
	// Accumulate user requested filters
	for (int i = 0; i < argc; ++i) {
		const char* a = argv[i];
		if (!strcmp(a, "--filter")) {
			if (++i != argc)
				filters.push_back(argv[i]);
		}
		else if (startswith(a, "--filter=")) {
			auto* ptr = strpbrk(a, "=");
			if (!ptr)
				continue;
			filters.push_back(ptr+1);
		}
	}
	
	return runTestsWithFilters(filters);
}

namespace commands {

	static void epicsTestRunAll(const iocshArgBuf* args) {
		const char* filters = args[0].sval;
		
		// Generate list of filters. For iocsh these will be semicolon separated
		std::vector<std::string> filterlist;
		if (filters) {
			char buf[2048];
			strncpy(buf, filters, sizeof(buf));
			for (char* s = strtok(buf, ";"); s; s = strtok(nullptr, ";")) {
				filterlist.push_back(s);
			}
		}
		
		if (runTestsWithFilters(filterlist) != 0 && s_testExitOnError) {
			epicsExit(1);
		}
	}
	
	static void epicsTestExitOnError(const iocshArgBuf* args) {
		s_testExitOnError = args[0].ival;
	}

}

void unit::registerCommands() {

	/* epicsTestRunAll(filters) */
	{
		static const iocshArg arg1 = {"Filters", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"epicsTestRunAll", 1, args, NULL};
		iocshRegister(&func, commands::epicsTestRunAll);
	}

	/* epicsTestExitOnError(filters) */
	{
		static const iocshArg arg1 = {"exitOnError", iocshArgString};
		static const iocshArg* const args[] = {&arg1};
		static const iocshFuncDef func = {"epicsTestExitOnError", 1, args, NULL};
		iocshRegister(&func, commands::epicsTestExitOnError);
	}

}

static bool supportsANSIColor() {
	static bool checked = false, value = false;
	if (checked)
		return value;
	checked = true;

#ifdef __linux__
	// Not a TTY; no escape codes
	if (!isatty(fileno(stdout))) {
		value = false;
		return value;
	}
	
	auto* env = getenv("TERM");
	if (env && !strcmp(env, "xterm-256color")) {
		value = true;
		return value;
	}
	
	env = getenv("COLORTERM"); // Try checking for truecolor next
	if (env && !strcmp(env, "truecolor")) {
		value = true;
		return value;
	}
	
	value = false;
	return false;
#else
	// For now this will remain FALSE
	// SetConsoleMode may be used to enable escape code support on Win10 16257 and later, but I have no way to test (for now)
	// See: https://learn.microsoft.com/en-us/windows/console/setconsolemode
	value = false;
	return false;
#endif
}


static void setStdoutColor(const char* color) {
	if (supportsANSIColor()) {
		printf("%s", color);
	}
}

std::string unit::to_string(char value) {
	char s[64];
	snprintf(s, sizeof(s), "%d", (int)value);
	return s;
}

std::string unit::to_string(unsigned char value) {
	char s[64];
	snprintf(s, sizeof(s), "%u", (uint32_t)value);
	return s;
}

std::string unit::to_string(void* ptr) {
	char s[64];
	if (!ptr)
		strcpy(s, "nullptr");
	else
		snprintf(s, sizeof(s), "%p", ptr);
	return s;
}

std::string unit::to_string(short value) {
	char s[64];
	snprintf(s, sizeof(s), "%d", (int)value);
	return s;
}

std::string unit::to_string(unsigned short value) {
	char s[64];
	snprintf(s, sizeof(s), "%u", (uint32_t)value);
	return s;
}
