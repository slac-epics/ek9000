TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Build an IOC support library
LIBRARY_IOC += ek9000Support

DBD += devEK9000.dbd
DBD += ek9000.dbd
DBD += ek9000Include.dbd

ifneq ($(NO_FORCE_CXX11),1)
USR_CXXFLAGS += --std=c++11
endif

ifeq ($(BUILD_STRICT),1)
USR_CXXFLAGS += -Wunused -Wall -Wextra -Wpedantic -Werror -Wno-deprecated-declarations -Wno-variadic-macros
endif

# Unfortunately EPICS and some of the motor record headers cause warnings (well, and one of our generated headers)
# So just disable those couple warnings to avoid spam (and so PEDANTIC=1 actually builds with -Werror
USR_CXXFLAGS += -Wno-deprecated-declarations -Wno-unused-variable

# Records/Device support sources
ek9000Support_SRCS += ekDiag.cpp
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
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# WARNING: Requires BASH or a similar shell 
clangdb_commands:
	echo -e "[\n" > compile_commands.json 
	for src in $(ek9000Support_SRCS); do \
		echo -e "\t{ \"directory\": \"$(shell pwd)\", " >> compile_commands.json; \
		echo -e "\t\"command\": \"-I$(MOTOR)/include -I$(ASYN)/include -I$(MODBUS)/include -I$(EPICS_BASE)/include $(USE_CXXFLAGS) -Dlinux -D_X86_ -DUNIX -D_GNU_SOURCE -D_DEFAULT_SOURCE\"," >> compile_commands.json; \
		echo -e "\t\"file\": \"$$src\"}," >> compile_commands.json; \
	done
	echo "{}]" >> compile_commands.json
