#!../../bin/$ARCH$/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

# Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# Configure the device (EK9K_NAME, IP, PORT, TERMINAL_COUNT)
ek9000Configure("${EK9K}", "${IP}", ${PORT}, ${NUM_TERMS})
asynSetTraceMask("${EK9K}", 0, 0x21) # 0x21 = (ASYN_TRACE_WARNING | ASYN_TRACE_ERROR)

${CONFIGURE}

cd "${TOP}/iocBoot/${IOC}"

$RECORDS$

dbLoadRecords("../../db/ek9000_status.db", "P=${PREFIX},EK9K=${EK9K}")

iocInit
