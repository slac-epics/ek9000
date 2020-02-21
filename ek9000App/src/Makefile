TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Build an IOC support library
LIBRARY_IOC += ek9000Support
USR_CXXFLAGS += --std=c++11

DBD += ek9000.dbd

# Records/Device support sources
ek9000Support_SRCS += devEK9000.cpp
ek9000Support_SRCS += devEL1XXX.cpp
ek9000Support_SRCS += devEL2XXX.cpp
ek9000Support_SRCS += devEL3XXX.cpp
ek9000Support_SRCS += devEL4XXX.cpp
#ek9000Support_SRCS += devEL7XXX.cpp

ek9000Support_LIBS += $(EPICS_BASE_IOC_LIBS)
ek9000Support_LIBS += modbus
ek9000Support_LIBS += asyn

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

