TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Build an IOC support library
LIBRARY_IOC += ek9000Support

DBD += devEK9000.dbd
DBD += ek9000.dbd
DBD += ek9000Include.dbd

ifneq ($(NO_FORCE_CXX03),1)
USR_CXXFLAGS += -std=c++03
else
USR_CXXFLAGS += -std=c++11
endif

ifeq ($(BUILD_STRICT),1)
USR_CXXFLAGS += -Wunused -Wall -Wextra -Wpedantic -Werror -Wno-deprecated-declarations -Wno-variadic-macros -Wno-unused-parameter
endif

ifeq ($(ENABLE_ASAN),1)
USR_CXXFLAGS += -fsanitize=address
USR_LDFLAGS += -fsanitize=address -static-libasan
endif

# Unfortunately EPICS and some of the motor record headers cause warnings (well, and one of our generated headers)
# So just disable those couple warnings to avoid spam (and so PEDANTIC=1 actually builds with -Werror
USR_CXXFLAGS += -Wno-deprecated-declarations -Wno-unused-variable

# Records/Device support sources
ek9000Support_SRCS += devEK9000.cpp
ek9000Support_SRCS += devEL1XXX.cpp
ek9000Support_SRCS += devEL2XXX.cpp
ek9000Support_SRCS += devEL3XXX.cpp
ek9000Support_SRCS += devEL4XXX.cpp
ek9000Support_SRCS += devEL50XX.cpp
ek9000Support_SRCS += ekUtil.cpp

ek9000Support_LIBS += $(EPICS_BASE_IOC_LIBS)
ek9000Support_LIBS += modbus
ek9000Support_LIBS += asyn

ifeq ($(ENABLE_MOTOR_SUPPORT),1)
ek9000Support_SRCS += devEL7XXX.cpp
ek9000Support_LIBS += motor
USR_CXXFLAGS += -DEK9000_MOTOR_SUPPORT=1
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE
