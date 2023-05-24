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

#include <epicsStdlib.h>

/**
 * @brief Decodes a EtherCAT diagnostics string
 * @param string Pointer to the diagnostics "string"
 * @param outbuf Output buffer
 * @param outbuflen Size of the output buffer
 * @return
 */
size_t COE_DecodeDiagString(void* string, char* outbuf, unsigned int outbuflen);
