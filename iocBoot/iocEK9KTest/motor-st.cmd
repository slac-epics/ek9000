#!../../bin/linux-arm/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# Configure the device
ek9000Configure("EK9K1", "10.0.0.3", 502, 5)
el70x7Configure("EK9K1","EK9K1_MOTOR_PORT","m1", 5)
ek9000ConfigureTerminal("EK9K1", "BO:1", "EL2008", 1)
ek9000ConfigureTerminal("EK9K1", "BI:1", "EL1004", 2)
ek9000ConfigureTerminal("EK9K1", "BO:2", "EL2008", 3)
ek9000ConfigureTerminal("EK9K1", "AI:1", "EL3064", 4)

dbLoadRecords("../motor/db/motorUtil.db", "P=IOC:m1")

cd "${TOP}/iocBoot/${IOC}"

dbLoadTemplate("motor.substitutions")

iocInit

#el70x7SetParam EK9K1_MOTOR_PORT maximal-current 100
