#!../../bin/$ARCH$/ek9000Test

< envPaths
dbLoadDatabase("../../dbd/ek9000Test.dbd")

cd "${TOP}"

# Register all support components
dbLoadDatabase "dbd/ek9000Test.dbd"
ek9000Test_registerRecordDeviceDriver pdbbase

# Configure the device (EK9K_NAME, IP, PORT, TERMINAL_COUNT)
ek9000Configure("EK9K1", "$IP$", $PORT$, $NUM_TERMS$)

$CONFIGURE$

cd "${TOP}/iocBoot/${IOC}"

$RECORDS$

iocInit
