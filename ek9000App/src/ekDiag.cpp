/*
 *
 * CoE diagnostics support
 *
 */ 
#include "diag.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

#pragma pack(1)
struct coe_diag_msg_t
{
	uint32_t diag_code;
	uint16_t flags;
	uint16_t textid;
	uint64_t timestamp;
	/* Bytes = 16 here */
	uint8_t params[15];
};
#pragma pack(0)

int COE_DecodeDiagString(void* string, char* outbuf, unsigned int outbuflen)
{
	coe_diag_msg_t msg = *(coe_diag_msg_t*)string;
	char buf[4096];

	/* Print out a timestamp */
	sprintf(buf, "%s", asctime(localtime((time_t*)&msg.timestamp)));
	
	//sprintf(buf, "%s Diagnostic code: %X, Flags: 0x%X, TextID: 0x%X\n",
	//		buf, msg.diag_code, msg.flags, msg.textid);
	//strncpy(outbuf, buf, outbuflen);
	//return outbuflen;
	/* Print in the type of message */
	switch((msg.textid & 0xF000) >> 11)
	{
		case 2:
			sprintf(buf, "%s [RESERVED]", buf);
			break;
		case 1:
			sprintf(buf, "%s [INFO]", buf);
			break;
		case 0:
			sprintf(buf, "%s [SYSINFO]", buf);
			break;
		case 4:
			sprintf(buf, "%s [WARN]", buf);
			break;
		default:
			sprintf(buf, "%s [ERROR]", buf);
	}

	/* Print in the subsystem */
	switch((msg.textid & 0x0F00) >> 7)
	{
		case 0:
			sprintf(buf, "%s [SYSTEM]", buf);
			break;
		case 1:
			sprintf(buf, "%s [GENERAL]", buf);
			break;
		case 2:
			sprintf(buf, "%s [COMM]", buf);
			break;
		case 3:
			sprintf(buf, "%s [ENC]", buf);
			break;
		case 4:
			sprintf(buf, "%s [DRIVE]", buf);
			break;
		case 5:
			sprintf(buf, "%s [INPUTS]", buf);
			break;
		case 6:
			sprintf(buf, "%s [I/O GEN]", buf);
			break;
		default:
			sprintf(buf, "%s [RESERVED]", buf);
			break;
	}

	/* Finally, a giant switch statement to handle simple message types */
	switch(msg.textid)
	{
		case 0x1:
			sprintf(buf, "%s No error", buf);
			break;
		case 0x2:
			sprintf(buf, "%s Communication established", buf);
			break;
		case 0x3:
			sprintf(buf, "%s Initialization: 0x%X, 0x%X, 0x%X", buf,
					msg.params[0], msg.params[1], msg.params[2]);
			break;
		case 0x1000:
			sprintf(buf, "%s Information: 0x%X, 0x%X, 0x%X", buf,
					msg.params[0], msg.params[1], msg.params[2]);
			break;
		case 0x1012:
			sprintf(buf, "%s EtherCAT state change Init - PreOP", buf);
			break;
		case 0x1021:
			sprintf(buf, "%s EtherCAT state change PreOP - Init", buf);
			break;
		case 0x1024:
			sprintf(buf, "%s EtherCAT state change PreOP - SafeOP", buf);
			break;
		case 0x1042:
			sprintf(buf, "%s EtherCAT state change SafeOP - PreOP", buf);
			break;
		case 0x1048:
			sprintf(buf, "%s EtherCAT state change SafeOP - OP", buf);
			break;
		case 0x1084:
			sprintf(buf, "%s EtherCAT state change OP - SafeOP", buf);
			break;
		case 0x1100:
			sprintf(buf, "%s Detection of operation mode completed: 0x%X, %d", buf,
					msg.params[0], (uint32_t)msg.params[5]);
			break;
		case 0x1135:
			sprintf(buf, "%s Cycle time OK: %d", buf, (uint32_t)msg.params[0]);
			break;
		case 0x1157:
			sprintf(buf, "%s Data manually saved (Idx: 0x%X, Subidx: 0x%X)", buf, msg.params[0], msg.params[1]);
			break;
		case 0x1158:
			sprintf(buf, "%s Data automatically saved (Idx: 0x%X, Subidx: 0x%X)", buf, msg.params[0], msg.params[1]);
			break;
		case 0x1159:
			sprintf(buf, "%s Data deleted (Idx: 0x%X, Subidx: 0x%X)", buf, msg.params[0], msg.params[1]);
			break;
		case 0x117F:
			sprintf(buf, "%s Information: 0x%X, 0x%X, 0x%X", buf, msg.params[0], msg.params[1], msg.params[2]);
			break;
		case 0x1201:
			sprintf(buf, "%s Communication re-established", buf);
			break;
		case 0x1300:
			sprintf(buf, "%s Position set: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[4]);
			break;
		case 0x1303:
			sprintf(buf, "%s Encoder supply OK", buf);
			break;
		case 0x1304:
			sprintf(buf, "%s Encoder initialization successful, channel: 0x%X", buf, msg.params[0]);
			break;
		case 0x1305:
			sprintf(buf, "%s Sent command encoder reset, channel: %X", buf, msg.params[0]);
			break;
		case 0x1400:
			sprintf(buf, "%s Drive is calibrated: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[4]);
			break;
		case 0x1401:
			sprintf(buf, "%s Actual drive state: 0x%X, %d", buf, msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x1705:
			sprintf(buf, "%s CPU usage returns in the normal range (<85%%)", buf);
			break;
		case 0x1706:
			sprintf(buf, "%s Channel is no longer saturated", buf);
			break;
		case 0x1707:
			sprintf(buf, "%s Channel is not overloaded anymore", buf);
			break;
		case 0x170A:
			sprintf(buf, "%s No channel range error anymore", buf);
			break;
		case 0x170C:
			sprintf(buf, "%s Calibration data saved", buf);
			break;
		case 0x170D:
			sprintf(buf, "%s Calibration data will be applied and saved after sending the command 0x5AFE", buf);
			break;
		case 0x2000:
			sprintf(buf, "%s Converting this command to a string is not supported", buf);
			break;
		case 0x2001:
			sprintf(buf, "%s Network link lost", buf);
			break;
		case 0x2002:
			sprintf(buf, "%s Network link detected", buf);
			break;
		case 0x2003:
			sprintf(buf, "%s No valid IP configuration found: DHCP client started.", buf);
			break;
		case 0x2004: 
			sprintf(buf, "%s valid IP configuration found", buf);
			break;
		case 0x2005:
			sprintf(buf, "%s DHCP client timed out", buf);
			break;
		case 0x2006:
			sprintf(buf, "%s Duplicate IP address detected", buf);
			break;
		case 0x2007:
			sprintf(buf, "%s UDP handler initialized", buf);
			break;
		case 0x2008:
			sprintf(buf, "%s TCP handler initialized", buf);
			break;
		case 0x2009:
			sprintf(buf, "%s No more TCP sockets available", buf);
			break;
		case 0x4001:
		case 0x4000:
		case 0x417F:
			sprintf(buf, "%s Warning: 0x%X, 0x%X, 0x%X", buf, msg.params[0], msg.params[1], msg.params[2]);
			break;
		case 0x4002:
			sprintf(buf, "%s Connection open", buf);
			break;
		case 0x4003:
			sprintf(buf, "%s Connection closed", buf);
			break;
		case 0x4004:
			sprintf(buf, "%s Connection timed out", buf);
			break;
		case 0x4005:
		case 0x4006:
		case 0x4007:
		case 0x4008:
			sprintf(buf, "%s Connection attempt deinied", buf);
			break;
		case 0x4101:
			sprintf(buf, "%s Terminal overtemp", buf);
			break;
		case 0x1402:
			sprintf(buf, "%s Discrepency in PDO conf", buf);
			break;
		case 0x428D:
			sprintf(buf, "%s Challenge is not random", buf);
			break;
		case 0x4300:
			sprintf(buf, "%s Subincrements deactivated: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[4]);
			break;
		case 0x4301:
			sprintf(buf, "%s Encoder warning", buf);
			break;
		case 0x4400:
			sprintf(buf, "%s Drive is no calibrated: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[4]);
			break;
		case 0x4401:
			sprintf(buf, "%s Starttype not supported: 0x%X, %d", buf, msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x4402:
			sprintf(buf, "%s Command rejected: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x4405:
			sprintf(buf, "%s Invalid modulo subtype: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[4]);
			break;
		case 0x4410:
			sprintf(buf, "%s Target overrun: %d, %d", buf, (uint32_t)msg.params[0], (uint32_t)msg.params[4]);
			break;
		case 0x4411:
			sprintf(buf, "%s DC-Link undervoltage", buf);
			break;
		case 0x4412:
			sprintf(buf, "%s DC-Link overvoltage", buf);
			break;
		case 0x4413:
			sprintf(buf, "%s I2T-Model Amplifier overload", buf);
			break;
		case 0x4414:
			sprintf(buf, "%s I2T-Model motor overload", buf);
			break;
		case 0x4415:
			sprintf(buf, "%s Speed limitation active", buf);
			break;
		case 0x4416:
			sprintf(buf, "%s Step loss detected at position: 0x%X%X", buf, msg.params[0], msg.params[1]);
			break;
		case 0x4417:
			sprintf(buf, "%s Motor overtemperature", buf);
			break;
		case 0x4418:
			sprintf(buf, "%s Current is limited", buf);
			break;
		case 0x4419:
			sprintf(buf, "%s Limit: Amplifier I2T model exceeds 100%%", buf);
			break;
		case 0x441A:
			sprintf(buf, "%s Limit: Motor I2T-model exceeds 100%%", buf);
			break;
		case 0x441B:
			sprintf(buf, "%s Limit: Velocity limit", buf);
			break;
		case 0x441C:
			sprintf(buf, "%s STO while axis was enabled", buf);
			break;
		case 0x4600:
			sprintf(buf, "%s Wrong supply voltage range", buf);
			break;
		case 0x4610:
			sprintf(buf, "%s Wrong output voltage range", buf);
			break;
		case 0x4705:
			sprintf(buf, "%s Processor usage at %d%%", buf, (uint32_t)msg.params[0]);
			break;
		case 0x470A:
			sprintf(buf, "%s EtherCAT frame missed", buf);
			break;
		case 0x8000:
			sprintf(buf, "%s %s", buf, msg.params);
			break;
		case 0x8001:
			sprintf(buf, "%s Error: 0x%X, 0x%X, 0x%X", buf, msg.params[0], msg.params[1], msg.params[2]);
			break;
		case 0x8002:
			sprintf(buf, "%s Communication aborted", buf);
			break;
		case 0x8003:
			sprintf(buf, "%s Configuration error: 0x%X, 0x%X, 0x%X", buf, msg.params[0], msg.params[1], msg.params[2]);
			break;
		case 0x8004:
			sprintf(buf, "%s %s: Unsuccessful FwdOpen-Response received", buf, msg.params);
			break;
		case 0x8005:
			sprintf(buf, "%s %s: FwdClose-Request sent", buf, msg.params);
			break;
		case 0x8006:
			sprintf(buf, "%s %s: Unsuccessful FwdClose-Response received", buf, msg.params);
			break;
		case 0x8007:
			sprintf(buf, "%s %s: Connection closed", buf, msg.params);
			break;
		case 0x8100:
			sprintf(buf, "%s Status word set: 0x%X,%d", buf, msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x8101:
			sprintf(buf, "%s Operation mode incompatible to PDO interface: 0x%X, %d", buf, msg.params[0], (uint32_t)msg.params[1]);
			break;
		case 0x8102:
			sprintf(buf, "%s Invalid combination of input and output PDOs", buf);
			break;
		case 0x8103:
			sprintf(buf, "%s No variable linkage", buf);
			break;
		case 0x8104:
			sprintf(buf, "%s Terminal overtemp", buf);
			break;
		case 0x8105:
			sprintf(buf, "%s PD-Watchdog", buf);
			break;
		case 0x8135:
			sprintf(buf, "%s Cycle time must be a multiple of 125us", buf);
			break;
		case 0x8136:
			sprintf(buf, "%s Configuration error: invalid sample rate", buf);
			break;
		case 0x8137:
			sprintf(buf, "%s Electronic type plate: CRC error", buf);
			break;
		case 0x8140:
			sprintf(buf, "%s Sync error", buf);
			break;
		case 0x8141:
			sprintf(buf, "%s Sync %X interrupt lost", buf, msg.params[0]);
			break;
		case 0x8142:
			sprintf(buf, "%s Sync interrupt async", buf);
			break;
		case 0x8143:
			sprintf(buf, "%s Jitter too big", buf);
			break;
		case 0x817F:
			sprintf(buf, "%s Error: 0x%X, 0x%X,0 0x%X", buf, msg.params[0], msg.params[1], msg.params[1]);
			break;
		default: break;
	}
	strncpy(outbuf, buf, outbuflen);
	return outbuflen;
}
