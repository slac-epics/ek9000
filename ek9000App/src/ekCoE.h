/*
 * This file is part of the EK9000 device support module. It is subject to 
 * the license terms in the LICENSE.txt file found in the top-level directory 
 * of this distribution and at: 
 *    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
 * No part of the EK9000 device support module, including this file, may be 
 * copied, modified, propagated, or distributed except according to the terms 
 * contained in the LICENSE.txt file.
*/
//======================================================//
// Name: devEL7XXX.cpp
// Purpose: Device support for EL7xxx modules (motor control)
//          requires the motor record module for epics
// Authors: Jeremy L.
// Date Created: July 31, 2020
//======================================================//
#pragma once

#include <epicsStdlib.h>

namespace coe
{
	enum EParamType
	{
		INT8 = 0,
		UINT8,
		INT16,
		UINT16,
		INT32,
		UINT32,
		INT64,
		UINT64,
		FLOAT32,
		FLOAT64,
		BOOL,
		STRING
	};

	struct param_t
	{
		const char* name;
		EParamType type;
		unsigned short index;
		unsigned short subindex;
		unsigned int length; // only for string params
	};

	// TODO
	//param_t FindParamByName(const char* name);
	//param_t FindParamByIndex(unsigned short index, unsigned short subindex);
}

