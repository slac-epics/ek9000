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
enum EParamType {
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

struct param_t {
	const char* name;
	EParamType type;
	unsigned short index;
	unsigned short subindex;
	unsigned int length; // only for string params
};


#define COE_PARAMETER(_name, _index, _subindex)                                                                        \
	CONSTEXPR uint16_t _name##_INDEX = _index;                                                                         \
	CONSTEXPR uint16_t _name##_SUBINDEX = _subindex

/** EL704X Parameters **/
COE_PARAMETER(EL704X_SPEED_RANGE, 0x8012, 0x5);
COE_PARAMETER(EL704X_VELOCITY_MIN, 0x8020, 0x1);
COE_PARAMETER(EL704X_ACCELERATION_POS, 0x8020, 0x3);
COE_PARAMETER(EL704X_DECELERATION_POS, 0x8020, 0x5);
COE_PARAMETER(EL704X_MAXIMAL_CURRENT, 0x8010, 0x1);
COE_PARAMETER(EL704X_REDUCED_CURRENT, 0x8010, 0x2);
COE_PARAMETER(EL704X_NOMINAL_VOLTAGE, 0x8010, 0x3);
COE_PARAMETER(EL704X_MOTOR_COIL_RESISTANCE, 0x8010, 0x4);
COE_PARAMETER(EL704X_MOTOR_EMF, 0x8010, 0x5);
COE_PARAMETER(EL704X_MOTOR_FULLSTEPS, 0x8010, 0x6);
COE_PARAMETER(EL704X_ENCODER_INCREMENTS, 0x8010, 0x7);
COE_PARAMETER(EL704X_START_VELOCITY, 0x8010, 0x9);
COE_PARAMETER(EL704X_MOTOR_COIL_INDUCTANCE, 0x8010, 0xA);
COE_PARAMETER(EL704X_DRIVE_ON_DELAY, 0x8010, 0x10);
COE_PARAMETER(EL704X_DRIVE_OFF_DELAY, 0x8010, 0x11);

} // namespace coe
