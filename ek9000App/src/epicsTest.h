/**
 * Minimal testing setup for EPICS device support modules
 * TODO: Split this out of here eventually?
 */
#pragma once

#include <iocsh.h>
#include <iocshRegisterCommon.h>
#include <stdint.h>
#include <string>
#include <type_traits>

#if __cplusplus >= 202002L
#define UNIT_CXX20 1
#endif

namespace unit {

	using std::to_string;
	
	std::string to_string(char value);
	std::string to_string(unsigned char value);
	std::string to_string(void* ptr);
	std::string to_string(short value);
	std::string to_string(unsigned short value);
	inline std::string to_string(std::nullptr_t value) { return "nullptr"; }
	inline std::string to_string(const std::string& value) { return value; }
	inline std::string to_string(const char* v) { return v; }
}

class UnitTest {
public:
	typedef void(*testProc)(UnitTest*);
	
	UnitTest() = delete;
	UnitTest(const char* test, const char* testSuite, testProc testProc);
	
	template<typename A, typename B>
	inline bool testAssertEq(A&& a, B&& b, const char* file, int line, bool fatal) {
		return testAssert(a == b, formatMsg(a,b,"=="), file, line, fatal);
	}
	
	template<typename A, typename B>
	inline bool testAssertNeq(A&& a, B&& b, const char* file, int line, bool fatal) {
		return testAssert(a != b, formatMsg(a,b,"!="), file, line, fatal);
	}
	
	template<typename A, typename B>
	inline bool testAssertGT(A&& a, B&& b, const char* file, int line, bool fatal) {
		return testAssert(a > b, formatMsg(a,b,">"), file, line, fatal);
	}
	
	template<typename A, typename B>
	inline bool testAssertGTE(A&& a, B&& b, const char* file, int line, bool fatal) {
		return testAssert(a >= b, formatMsg(a,b,">="), file, line, fatal);
	}
	
	template<typename A, typename B>
	inline bool testAssertLT(A&& a, B&& b, const char* file, int line, bool fatal) {
		return testAssert(a < b, formatMsg(a,b,"<"), file, line, fatal);
	}
	
	template<typename A, typename B>
	inline bool testAssertLTE(A&& a, B&& b, const char* file, int line, bool fatal) {
		return testAssert(a <= b, formatMsg(a,b,"<="), file, line, fatal);
	}
	
	/**
	 * @brief End of test reporting. Displays success/failure, number of asserts, number of failed
	 */
	void report();
	
	std::string fullName() const;
	
	bool exec();
	
	void reset();
	
	uint32_t asserts() const { return asserts_; }
	uint32_t failedAsserts() const { return failed_; }
	uint32_t passedAsserts() const { return asserts_ - failed_; }
	
private:

	bool testAssert(bool cond, const std::string& failMsg, const char* file, int line, bool fatal);

	template<class A, class B>
	std::string formatMsg(A&& a, B&& b, const char* op) {
		char msg[16384];
		snprintf(msg, sizeof(msg), "%s %s %s", unit::to_string(a).c_str(), op, unit::to_string(b).c_str());
		return msg;
	}

	testProc proc_;
	const std::string name_;
	const std::string desc_;
	uint32_t asserts_;
	uint32_t failed_;
};

namespace unit
{
	/**
	 * Call this from a registrar function to expose iocsh commands for interacting with the test system
	 */
	void registerCommands();
	
	/**
	 * Call this from main() or somewhere else where you want to execute commands
	 * Alternatively, you may not call this and instead run tests from inside of IOC shell
	 * using epicsTestRunAll
	 */
	int runTests(int argc, char** argv);
}

#define EPICS_TEST(test, testSuite) \
	void __func_##test##_exec (UnitTest* unitTest); \
	UnitTest __s_##test (#test, testSuite, __func_##test##_exec); \
	void __func_##test##_exec (UnitTest* unitTest)


#define _EPICS_TEST_ASSERT(tp, a, b) do{ auto _a = (a); auto _b = (b); if(!unitTest->tp((_a), (_b), __FILE__, __LINE__, true)) return; } while(0)
#define _EPICS_TEST_EXPECT(tp, a, b) do{ auto _a = (a); auto _b = (b); if(!unitTest->tp((_a), (_b), __FILE__, __LINE__, false)) return; } while(0)

// Assertions will abort the test. Use these for constraints that should be considered fatal or may crash the application
// Expect should be preferred for non-fatal test cases
#define ASSERT_TRUE(a) _EPICS_TEST_ASSERT(testAssertEq, (a), true)
#define ASSERT_FALSE(a) _EPICS_TEST_ASSERT(testAssertEq, (a), false)
#define ASSERT_EQ(a, b) _EPICS_TEST_ASSERT(testAssertEq, (a), (b))
#define ASSERT_NEQ(a, b) _EPICS_TEST_ASSERT(testAssertNeq, (a), (b))
#define ASSERT_GT(a, b) _EPICS_TEST_ASSERT(testAssertGT, (a), (b))
#define ASSERT_GTE(a, b) _EPICS_TEST_ASSERT(testAssertGTE, (a), (b))
#define ASSERT_LT(a, b) _EPICS_TEST_ASSERT(testAssertLT, (a), (b))
#define ASSERT_LTE(a, b) _EPICS_TEST_ASSERT(testAssertLTE, (a), (b))
#define ASSERT_NOT_NULL(a) _EPICS_TEST_ASSERT(testAssertNeq, (a), nullptr)


#define EXPECT_TRUE(a) _EPICS_TEST_EXPECT(testAssertEq, (a), true)
#define EXPECT_FALSE(a) _EPICS_TEST_EXPECT(testAssertEq, (a), false)
#define EXPECT_EQ(a, b) _EPICS_TEST_EXPECT(testAssertEq, (a), (b))
#define EXPECT_NEQ(a, b) _EPICS_TEST_EXPECT(testAssertNeq, (a), (b))
#define EXPECT_GT(a, b) _EPICS_TEST_EXPECT(testAssertGT, (a), (b))
#define EXPECT_GTE(a, b) _EPICS_TEST_EXPECT(testAssertGTE, (a), (b))
#define EXPECT_LT(a, b) _EPICS_TEST_EXPECT(testAssertLT, (a), (b))
#define EXPECT_LTE(a, b) _EPICS_TEST_EXPECT(testAssertLTE, (a), (b))
#define EXPECT_NOT_NULL(a) _EPICS_TEST_EXPECT(testAssertNeq, (a), nullptr)
