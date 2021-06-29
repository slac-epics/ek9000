/*
 *
 * CoE diagnostics support
 *
 */
#include "ekDiag.h"
#include <stdint.h>
#include <string.h>
#include <epicsString.h>
#include <epicsTime.h>

#pragma pack(1)
struct coe_diag_msg_t {
	uint32_t diag_code;
	uint16_t flags;
	uint16_t textid;
	uint64_t timestamp;
	/* Bytes = 16 here */
	uint8_t params[15];
};
#pragma pack(0)

size_t COE_DecodeDiagString(void* string, char* outbuf, unsigned int outbuflen) {
	coe_diag_msg_t msg = *(coe_diag_msg_t*)string;
	char buf[8192];
	char tmpbuf[512];

	/* Print out a timestamp */
	snprintf(tmpbuf, sizeof(tmpbuf), "%s", asctime(localtime((time_t*)&msg.timestamp)));

	/* Print in the type of message */
	switch ((msg.textid & 0xF000) >> 11) {
		case 2:
			snprintf(buf, sizeof(buf), "%s [RESERVED]", tmpbuf);
			break;
		case 1:
			snprintf(buf, sizeof(buf), "%s [INFO]", tmpbuf);
			break;
		case 0:
			snprintf(buf, sizeof(buf), "%s [SYSINFO]", tmpbuf);
			break;
		case 4:
			snprintf(buf, sizeof(buf), "%s [WARN]", tmpbuf);
			break;
		default:
			snprintf(buf, sizeof(buf), "%s [ERROR]", tmpbuf);
	}

	/* Print in the subsystem */
	switch ((msg.textid & 0x0F00) >> 7) {
		case 0:
			snprintf(buf, sizeof(buf), "%s [SYSTEM]", tmpbuf);
			break;
		case 1:
			snprintf(buf, sizeof(buf), "%s [GENERAL]", tmpbuf);
			break;
		case 2:
			snprintf(buf, sizeof(buf), "%s [COMM]", tmpbuf);
			break;
		case 3:
			snprintf(buf, sizeof(buf), "%s [ENC]", tmpbuf);
			break;
		case 4:
			snprintf(buf, sizeof(buf), "%s [DRIVE]", tmpbuf);
			break;
		case 5:
			snprintf(buf, sizeof(buf), "%s [INPUTS]", tmpbuf);
			break;
		case 6:
			snprintf(buf, sizeof(buf), "%s [I/O GEN]", tmpbuf);
			break;
		default:
			snprintf(buf, sizeof(buf), "%s [RESERVED]", tmpbuf);
			break;
	}

	/* Finally, a giant switch statement to handle simple message types */
	switch (msg.textid) {
		case 0x1:
			snprintf(buf, sizeof(buf), "%s No error", tmpbuf);
			break;
		case 0x2:
			snprintf(buf, sizeof(buf), "%s Communication established", tmpbuf);
			break;
		case 0x3:
			snprintf(buf, sizeof(buf), "%s Initialization: 0x%X, 0x%X, 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[2]);
			break;
		case 0x1000:
			snprintf(buf, sizeof(buf), "%s Information: 0x%X, 0x%X, 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[2]);
			break;
		case 0x1012:
			snprintf(buf, sizeof(buf), "%s EtherCAT state change Init - PreOP", tmpbuf);
			break;
		case 0x1021:
			snprintf(buf, sizeof(buf), "%s EtherCAT state change PreOP - Init", tmpbuf);
			break;
		case 0x1024:
			snprintf(buf, sizeof(buf), "%s EtherCAT state change PreOP - SafeOP", tmpbuf);
			break;
		case 0x1042:
			snprintf(buf, sizeof(buf), "%s EtherCAT state change SafeOP - PreOP", tmpbuf);
			break;
		case 0x1048:
			snprintf(buf, sizeof(buf), "%s EtherCAT state change SafeOP - OP", tmpbuf);
			break;
		case 0x1084:
			snprintf(buf, sizeof(buf), "%s EtherCAT state change OP - SafeOP", tmpbuf);
			break;
		case 0x1100:
			snprintf(buf, sizeof(buf), "%s Detection of operation mode completed: 0x%X, %d", tmpbuf, msg.params[0],
					 (uint32_t)msg.params[5]);
			break;
		case 0x1135:
			snprintf(buf, sizeof(buf), "%s Cycle time OK: %d", tmpbuf, (uint32_t)msg.params[0]);
			break;
		case 0x1157:
			snprintf(buf, sizeof(buf), "%s Data manually saved (Idx: 0x%X, Subidx: 0x%X)", tmpbuf, msg.params[0],
					 msg.params[1]);
			break;
		case 0x1158:
			snprintf(buf, sizeof(buf), "%s Data automatically saved (Idx: 0x%X, Subidx: 0x%X)", tmpbuf, msg.params[0],
					 msg.params[1]);
			break;
		case 0x1159:
			snprintf(buf, sizeof(buf), "%s Data deleted (Idx: 0x%X, Subidx: 0x%X)", tmpbuf, msg.params[0],
					 msg.params[1]);
			break;
		case 0x117F:
			snprintf(buf, sizeof(buf), "%s Information: 0x%X, 0x%X, 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[2]);
			break;
		case 0x1201:
			snprintf(buf, sizeof(buf), "%s Communication re-established", tmpbuf);
			break;
		case 0x1300:
			snprintf(buf, sizeof(buf), "%s Position set: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[4]);
			break;
		case 0x1303:
			snprintf(buf, sizeof(buf), "%s Encoder supply OK", tmpbuf);
			break;
		case 0x1304:
			snprintf(buf, sizeof(buf), "%s Encoder initialization successful, channel: 0x%X", tmpbuf, msg.params[0]);
			break;
		case 0x1305:
			snprintf(buf, sizeof(buf), "%s Sent command encoder reset, channel: %X", tmpbuf, msg.params[0]);
			break;
		case 0x1400:
			snprintf(buf, sizeof(buf), "%s Drive is calibrated: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[4]);
			break;
		case 0x1401:
			snprintf(buf, sizeof(buf), "%s Actual drive state: 0x%X, %d", tmpbuf, msg.params[0],
					 (uint32_t)msg.params[1]);
			break;
		case 0x1705:
			snprintf(buf, sizeof(buf), "%s CPU usage returns in the normal range (<85%%)", tmpbuf);
			break;
		case 0x1706:
			snprintf(buf, sizeof(buf), "%s Channel is no longer saturated", tmpbuf);
			break;
		case 0x1707:
			snprintf(buf, sizeof(buf), "%s Channel is not overloaded anymore", tmpbuf);
			break;
		case 0x170A:
			snprintf(buf, sizeof(buf), "%s No channel range error anymore", tmpbuf);
			break;
		case 0x170C:
			snprintf(buf, sizeof(buf), "%s Calibration data saved", tmpbuf);
			break;
		case 0x170D:
			snprintf(buf, sizeof(buf), "%s Calibration data will be applied and saved after sending the command 0x5AFE",
					 tmpbuf);
			break;
		case 0x2000:
			snprintf(buf, sizeof(buf), "%s Converting this command to a string is not supported", tmpbuf);
			break;
		case 0x2001:
			snprintf(buf, sizeof(buf), "%s Network link lost", tmpbuf);
			break;
		case 0x2002:
			snprintf(buf, sizeof(buf), "%s Network link detected", tmpbuf);
			break;
		case 0x2003:
			snprintf(buf, sizeof(buf), "%s No valid IP configuration found: DHCP client started.", tmpbuf);
			break;
		case 0x2004:
			snprintf(buf, sizeof(buf), "%s valid IP configuration found", tmpbuf);
			break;
		case 0x2005:
			snprintf(buf, sizeof(buf), "%s DHCP client timed out", tmpbuf);
			break;
		case 0x2006:
			snprintf(buf, sizeof(buf), "%s Duplicate IP address detected", tmpbuf);
			break;
		case 0x2007:
			snprintf(buf, sizeof(buf), "%s UDP handler initialized", tmpbuf);
			break;
		case 0x2008:
			snprintf(buf, sizeof(buf), "%s TCP handler initialized", tmpbuf);
			break;
		case 0x2009:
			snprintf(buf, sizeof(buf), "%s No more TCP sockets available", tmpbuf);
			break;
		case 0x4001:
		case 0x4000:
		case 0x417F:
			snprintf(buf, sizeof(buf), "%s Warning: 0x%X, 0x%X, 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[2]);
			break;
		case 0x4002:
			snprintf(buf, sizeof(buf), "%s Connection open", tmpbuf);
			break;
		case 0x4003:
			snprintf(buf, sizeof(buf), "%s Connection closed", tmpbuf);
			break;
		case 0x4004:
			snprintf(buf, sizeof(buf), "%s Connection timed out", tmpbuf);
			break;
		case 0x4005:
		case 0x4006:
		case 0x4007:
		case 0x4008:
			snprintf(buf, sizeof(buf), "%s Connection attempt deinied", tmpbuf);
			break;
		case 0x4101:
			snprintf(buf, sizeof(buf), "%s Terminal overtemp", tmpbuf);
			break;
		case 0x1402:
			snprintf(buf, sizeof(buf), "%s Discrepency in PDO conf", tmpbuf);
			break;
		case 0x428D:
			snprintf(buf, sizeof(buf), "%s Challenge is not random", tmpbuf);
			break;
		case 0x4300:
			snprintf(buf, sizeof(buf), "%s Subincrements deactivated: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[4]);
			break;
		case 0x4301:
			snprintf(buf, sizeof(buf), "%s Encoder warning", tmpbuf);
			break;
		case 0x4400:
			snprintf(buf, sizeof(buf), "%s Drive is no calibrated: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[4]);
			break;
		case 0x4401:
			snprintf(buf, sizeof(buf), "%s Starttype not supported: 0x%X, %d", tmpbuf, msg.params[0],
					 (uint32_t)msg.params[1]);
			break;
		case 0x4402:
			snprintf(buf, sizeof(buf), "%s Command rejected: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[1]);
			break;
		case 0x4405:
			snprintf(buf, sizeof(buf), "%s Invalid modulo subtype: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[4]);
			break;
		case 0x4410:
			snprintf(buf, sizeof(buf), "%s Target overrun: %d, %d", tmpbuf, (uint32_t)msg.params[0],
					 (uint32_t)msg.params[4]);
			break;
		case 0x4411:
			snprintf(buf, sizeof(buf), "%s DC-Link undervoltage", tmpbuf);
			break;
		case 0x4412:
			snprintf(buf, sizeof(buf), "%s DC-Link overvoltage", tmpbuf);
			break;
		case 0x4413:
			snprintf(buf, sizeof(buf), "%s I2T-Model Amplifier overload", tmpbuf);
			break;
		case 0x4414:
			snprintf(buf, sizeof(buf), "%s I2T-Model motor overload", tmpbuf);
			break;
		case 0x4415:
			snprintf(buf, sizeof(buf), "%s Speed limitation active", tmpbuf);
			break;
		case 0x4416:
			snprintf(buf, sizeof(buf), "%s Step loss detected at position: 0x%X%X", tmpbuf, msg.params[0],
					 msg.params[1]);
			break;
		case 0x4417:
			snprintf(buf, sizeof(buf), "%s Motor overtemperature", tmpbuf);
			break;
		case 0x4418:
			snprintf(buf, sizeof(buf), "%s Current is limited", tmpbuf);
			break;
		case 0x4419:
			snprintf(buf, sizeof(buf), "%s Limit: Amplifier I2T model exceeds 100%%", tmpbuf);
			break;
		case 0x441A:
			snprintf(buf, sizeof(buf), "%s Limit: Motor I2T-model exceeds 100%%", tmpbuf);
			break;
		case 0x441B:
			snprintf(buf, sizeof(buf), "%s Limit: Velocity limit", tmpbuf);
			break;
		case 0x441C:
			snprintf(buf, sizeof(buf), "%s STO while axis was enabled", tmpbuf);
			break;
		case 0x4600:
			snprintf(buf, sizeof(buf), "%s Wrong supply voltage range", tmpbuf);
			break;
		case 0x4610:
			snprintf(buf, sizeof(buf), "%s Wrong output voltage range", tmpbuf);
			break;
		case 0x4705:
			snprintf(buf, sizeof(buf), "%s Processor usage at %d%%", tmpbuf, (uint32_t)msg.params[0]);
			break;
		case 0x470A:
			snprintf(buf, sizeof(buf), "%s EtherCAT frame missed", tmpbuf);
			break;
		case 0x8000:
			snprintf(buf, sizeof(buf), "%s %s", tmpbuf, msg.params);
			break;
		case 0x8001:
			snprintf(buf, sizeof(buf), "%s Error: 0x%X, 0x%X, 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[2]);
			break;
		case 0x8002:
			snprintf(buf, sizeof(buf), "%s Communication aborted", tmpbuf);
			break;
		case 0x8003:
			snprintf(buf, sizeof(buf), "%s Configuration error: 0x%X, 0x%X, 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[2]);
			break;
		case 0x8004:
			snprintf(buf, sizeof(buf), "%s %s: Unsuccessful FwdOpen-Response received", tmpbuf, msg.params);
			break;
		case 0x8005:
			snprintf(buf, sizeof(buf), "%s %s: FwdClose-Request sent", tmpbuf, msg.params);
			break;
		case 0x8006:
			snprintf(buf, sizeof(buf), "%s %s: Unsuccessful FwdClose-Response received", tmpbuf, msg.params);
			break;
		case 0x8007:
			snprintf(buf, sizeof(buf), "%s %s: Connection closed", tmpbuf, msg.params);
			break;
		case 0x8100:
			snprintf(buf, sizeof(buf), "%s Status word set: 0x%X,%d", tmpbuf, msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x8101:
			snprintf(buf, sizeof(buf), "%s Operation mode incompatible to PDO interface: 0x%X, %d", tmpbuf,
					 msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x8102:
			snprintf(buf, sizeof(buf), "%s Invalid combination of input and output PDOs", tmpbuf);
			break;
		case 0x8103:
			snprintf(buf, sizeof(buf), "%s No variable linkage", tmpbuf);
			break;
		case 0x8104:
			snprintf(buf, sizeof(buf), "%s Terminal overtemp", tmpbuf);
			break;
		case 0x8105:
			snprintf(buf, sizeof(buf), "%s PD-Watchdog", tmpbuf);
			break;
		case 0x8135:
			snprintf(buf, sizeof(buf), "%s Cycle time must be a multiple of 125us", tmpbuf);
			break;
		case 0x8136:
			snprintf(buf, sizeof(buf), "%s Configuration error: invalid sample rate", tmpbuf);
			break;
		case 0x8137:
			snprintf(buf, sizeof(buf), "%s Electronic type plate: CRC error", tmpbuf);
			break;
		case 0x8140:
			snprintf(buf, sizeof(buf), "%s Sync error", tmpbuf);
			break;
		case 0x8141:
			snprintf(buf, sizeof(buf), "%s Sync %X interrupt lost", tmpbuf, msg.params[0]);
			break;
		case 0x8142:
			snprintf(buf, sizeof(buf), "%s Sync interrupt async", tmpbuf);
			break;
		case 0x8143:
			snprintf(buf, sizeof(buf), "%s Jitter too big", tmpbuf);
			break;
		case 0x817F:
			snprintf(buf, sizeof(buf), "%s Error: 0x%X, 0x%X,0 0x%X", tmpbuf, msg.params[0], msg.params[1],
					 msg.params[1]);
			break;
		default:
			break;
	}
	strncpy(outbuf, tmpbuf, outbuflen);
	return outbuflen;
}
