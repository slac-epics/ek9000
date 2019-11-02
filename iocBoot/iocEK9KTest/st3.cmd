#!../../bin/linux-x86_64/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# Configure the device
ek9000Configure("EK9K1", "192.168.1.3", 502, 5)
el70x7Configure("EK9K1","EK9K1_MOTOR_PORT",5)
ek9000ConfigureTerminal("EK9K1", "TestTerm1-BO", "EL2008", 1)
ek9000ConfigureTerminal("EK9K1", "TestTerm2-BI", "EL1004", 2)
ek9000ConfigureTerminal("EK9K1", "TestTerm3-BO", "EL2008", 3)
ek9000ConfigureTerminal("EK9K1", "TestTerm4-AI", "EL3064", 4)

dbLoadRecords("../motor/db/motorUtil.db", "P=el70x7:")

cd "${TOP}/iocBoot/${IOC}"

dbLoadTemplate("Test3.substitutions")
dbLoadTemplate("motor.substitutions")

iocInit
