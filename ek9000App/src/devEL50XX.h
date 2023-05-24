/*
 * This file is part of the EK9000 device support module. It is subject to
 * the license terms in the LICENSE.txt file found in the top-level directory
 * of this distribution and at:
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
 * No part of the EK9000 device support module, including this file, may be
 * copied, modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */
#pragma once

#include <stdint.h>
#include <epicsAssert.h>

#pragma pack(1)
struct EL5001Status_t {
	uint8_t data_error : 1;
	uint8_t frame_error : 1;
	uint8_t power_fail : 1;
	uint8_t data_mismatch : 1;
	uint8_t _r1 : 1;
	uint8_t sync_err : 1;
	uint8_t txpdo_state : 1;
	uint8_t txpdo_toggle : 1;
};

STATIC_ASSERT(sizeof(EL5001Status_t) == sizeof(uint8_t));

/* Input data from an el5001 terminal */
struct EL5001Output_t {
	union {
		uint8_t status_byte;
		EL5001Status_t data;
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
