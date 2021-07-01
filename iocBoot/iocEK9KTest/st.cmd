#!../../bin/linux-x86_64/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# Configure the device (EK9K_NAME, IP, PORT, TERMINAL_COUNT)
ek9000Configure("EK9K1", "10.0.0.3", 502, 5)
ek9000EnableDebug("EK9K1")

# Configure each terminal (EK9K_ANME, RECORD_NAME, TERMINAL_TYPE, POSITION)
ek9000ConfigureTerminal("EK9K1", "BO:1", "EL2008", 1)
ek9000ConfigureTerminal("EK9K1", "BI:1", "EL1004", 2)
ek9000ConfigureTerminal("EK9K1", "RTD:1", "EL3314", 3)
ek9000ConfigureTerminal("EK9K1", "AI:1", "EL3064", 4)

cd "${TOP}/iocBoot/${IOC}"

dbLoadRecords("../../db/EL2008.template", "TERMINAL=BO:1")
dbLoadRecords("../../db/EL1004.template", "TERMINAL=BI:1")
dbLoadRecords("../../db/EL3314.template", "TERMINAL=RTD:1")
dbLoadRecords("../../db/EL3064.template", "TERMINAL=AI:1")

iocInit
