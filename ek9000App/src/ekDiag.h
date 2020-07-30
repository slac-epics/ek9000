/*
 *
 * Some diagnostics support for EtherCAT terminals
 *
 */ 
#pragma once

/* String constants for associated error numbers */
typedef struct
{
	unsigned int id;
	const char* msg;
	unsigned int extra_parms;
} coe_error_type_t;

static coe_error_type_t coe_error_types[] =
{
	{0x1, "No error", 0},
	{0x2, "Connection established", 0},
	{0x3, "Initialization 0x%X, 0x%X, 0x%X", 3}, //TODO: This has some additional params
	{0x1000, "Information 0x%X, 0x%X, 0x%X", 3}, //TODO: this has some additional params
	{0x1012, "EtherCAT state change Init - PreOP", 0},
	{0x1021, "EtherCAT state change PreOP - Init", 0},
	{0x1024, "EtherCAT state change PreOP - SafeOP", 0},
	{0x1042, "EtherCAT state change SafeOP - PreOP", 0},
	{0x1048, "EtherCAT state change SafeOP - OP", 0},
	{0x1084, "EtherCAT state change Op - SafeOP", 0},
	{0x1100, "Detection of operation mode completed: 0x%X, %d", 2},
	{0x1135, "Cycle time OK: %d", 1},
	{0x1157, "Data manually saved (Idx: 0x%X, Subidx: 0x%X)", 2},
	{0x1158, "Data automatically saved (Idx: 0x%X, Subidx: 0x%X)", 2},
	{0x1159, "Data deleted (Idx: 0x%X, Subidx: 0x%X)", 2},
	{0x117F, "Information: 0x%X, 0x%X, 0x%X", 3},
	{0x1201, "Communication re-established", 0},
	{0x1300, "Position set: %d, %d", 2},
	{0x1303, "Encoder supply OK", 0},
	{0x1304, "Encoder initialization successful, channel: %X", 1},
};


int COE_DecodeDiagString(void* string, char* outbuf, unsigned int outbuflen);


