#!../../bin/linux-x86_64/ek9000Test

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
ek9000Configure("EK9K1", "192.168.1.3", 502, 2)
# ek9000ConfigureTerminal takes 4 parameters: the name of the device which it's attached to, the record base name, the device type string, and the position on the rail.
#ek9000ConfigureTerminal("EK9K1", "TestTerm1", "EL2008", 1)
#ek9000ConfigureTerminal("EK9K1", "TestTerm2", "EL1004", 2)
#ek9000ConfigureTerminal("EK9K1", "TestTerm3", "EL2008", 3)
#ek9000ConfigureTerminal("EK9K1", "TestTerm4", "EL3064", 4)
#ek9000ConfigureTerminal("EK9K1", "TestTerm5", "EL3202", 5)
ek9000ConfigureTerminal("EK9K1", "RTD1", "EL3202", 1)
ek9000ConfigureTerminal("EK9K1", "RTD2", "EL3202", 2)
#ek9000ConfigureTerminal("EK9K1", "RTD3", "EL3202", 3)

dbLoadRecords("db/EL3202.template", "TERMINAL=RTD1")
dbLoadRecords("db/EL3202.template", "TERMINAL=RTD2")
#dbLoadRecords("db/EL3202.template", "TERMINAL=RTD3")

cd "${TOP}/src/ek9000/iocBoot/${IOC}"

# Load our example subs file 
#dbLoadTemplate("example1.substitutions")

#dbLoadRecords("db/coe_param_test.db")
#dbLoadRecords("db/EK9000_Status.db","EK9K=EK9K1")

iocInit
