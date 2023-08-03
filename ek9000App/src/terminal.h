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

// If you change this struct, be sure to update scripts/generate.py!
struct terminal_t {
	// String name of the terminal. e.g. EL3064
	const char* str;

	// Integer ID of the terminal. e.g. 3064 from EL3064
	uint32_t id;

	// Total size means "individual channel size * num channels".
	// If you have a 4 channel analog input terminal with 4-bytes input/status data per channel,
	// inputSize will be (4 * 4) / 2 = 8 registers. Note that sizes are either in registers or coils, NEVER in bytes.

	// Total input size of the POD, in registers. If this is a digital terminal,
	// this will be in coils instead.
	uint16_t inputSize;

	// Number of inputs
	uint16_t numInputs;

	// Total output size of the PDO, in registers. If this is a digital terminal,
	// this will be in coils instead
	uint16_t outputSize;

	// Number of outputs
	uint16_t numOutputs;
};
