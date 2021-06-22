/*
 *
 * device support for EL50XX encoders.
 *
 */
#pragma once

#include <stdint.h>

#pragma pack(1)
struct data_t {
	uint8_t data_error : 1;
	uint8_t frame_error : 1;
	uint8_t power_fail : 1;
	uint8_t data_mismatch : 1;
	uint8_t _r1 : 1;
	uint8_t sync_err : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
};

/* Input data from an el5001 terminal */
struct EL5001Output_t {
	union {
		uint8_t status_byte;
		data_t data;
	};
	uint32_t encoder_value;
};

/* Input data from an el5002 slave, given it has the extended status byte enabled */
struct EL5002Output_t {
	uint8_t data_error : 1;
	uint8_t frame_error : 1;
	uint8_t power_fail : 1;
	uint16_t _r1 : 10;
	uint8_t _r2 : 3;
	uint32_t encoder_value;
};

#pragma pack()
