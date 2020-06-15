TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Build an IOC support library
LIBRARY_IOC += ek9000Support

DBD += devEK9000.dbd
DBD += ek9000.dbd
DBD += ek9000Include.dbd

USR_CXXFLAGS += --std=c++17 -g

# Used for code improvement purposes
#USR_CXXFLAGS += -Wunused -Wall -Wextra -Wpedantic

# Records/Device support sources
ek9000Support_SRCS += diag.cpp
ek9000Support_SRCS += drvEK9000.cpp
ek9000Support_SRCS += util.cpp

ek9000Support_LIBS += $(EPICS_BASE_IOC_LIBS)
ek9000Support_LIBS += modbus
ek9000Support_LIBS += asyn
ek9000Support_LIBS += motor

#PROD_IOC = ek9000

#
#============================================
# IOC stuff
#============================================
# ek9000.dbd will include these files:

#ek9000_DBD += base.dbd
#ek9000_DBD += ek9kDevices.dbd
#ek9000_DBD += ek9000Include.dbd
#ek9000_DBD += ek9kRecord.dbd

# ek9000_registerRecordDeviceDriver.cpp derives from ek9000.dbd
#ek9000_SRCS += ek9000_registerRecordDeviceDriver.cpp
#ek9000_SRCS += ek9kDevices_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
#ek9000_SRCS_DEFAULT += ek9000Main.cpp
#ek9000_SRCS_vxWorks += -nil-

# Link in the code from our support library
#ek9000_LIBS += ek9000Support

# Finally link IOC to the EPICS Base libraries
#ek9000_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

