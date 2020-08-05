#!../../bin/linux-arm/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# My testing rail looks like this:
# slave #      1         2         3        4
# ek9000 -> el2008 -> el1004 -> el2008 -> el3064 -> end of the rail

# Configure the device
# ek9000Configure takes 4 parameters: your *unique* device name, the IP of the device, the port number and the number of slaves you wish to access
drvAsynIPPortConfigure("EK9K1_PORT", "192.168.1.3:502", 0, 0, 1)
modbusInterposeConfig("EK9K1_PORT", 0, 2000, 0)
drvModbusAsynConfigure("EK9K1", "EK9K1_PORT", 0,4,1,10,2,100,"Cheese")
ek9000Register("EK9K1D", "EK9K1", "192.168.1.3:502")

cd "${TOP}/iocBoot/${IOC}"

# Load our example subs file 
#dbLoadTemplate("example1.substitutions")

iocInit
