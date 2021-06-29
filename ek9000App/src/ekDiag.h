/*
 *
 * Some diagnostics support for EtherCAT terminals
 *
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
