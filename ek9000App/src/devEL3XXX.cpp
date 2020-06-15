/*
 *
 * devEL3XXX.cpp
 *
 * Analog input terminals
 *
 */ 

#include <aiRecord.h>
#include <epicsSpin.h>
#include <epicsStdio.h>

#include "util.h"
#include "drvEK9000.h"

static long el30xx_read_record(void* precord);
static long el30xx_init_record(void* precord);
static long el33xx_read_record(void* precord);
static long el33xx_init_record(void* precord);

/*
 * Device support for basic analog input records with 
 * 32-bits per channel in standard mode, 16-bit status, 
 * 16-bit adc count.
 * this also supports compact PDO mode, with 16-bits per channel,
 * no status, only adc counts
 */ 
struct {
	long        number;
	DEVSUPFUN   report;
	DEVSUPFUN   init;
	DEVSUPFUN   init_record;
	DEVSUPFUN   get_ioint_info;
	DEVSUPFUN   read_ai;
	DEVSUPFUN   special_linconv;
} devEL30XX = {
	6,
	NULL,
	NULL,
	el30xx_init_record,
	NULL,
	el30xx_read_record,
	NULL
};

/*
 * Device support for EL33XX thermocouple modules
 * These have a total of 48-bits per channel
 */ 
struct {
    long        number;
    DEVSUPFUN   report;
    DEVSUPFUN   init;
    DEVSUPFUN   init_record;
    DEVSUPFUN   get_ioint_info;
    DEVSUPFUN   read_ai;
    DEVSUPFUN   special_linconv;
} devEL33XX = {
    6,
    NULL,
    NULL,
    el33xx_init_record,
    NULL,
    el33xx_read_record,
    NULL
};

static long el30xx_init_record(void* precord)
{
	aiRecord* prec = static_cast<aiRecord*>(precord);

	prec->dpvt = util::parseAndCreateDpvt(prec->inp.value.instio.string);

	return prec->dpvt ? 0 : 1;
}

static long el30xx_read_record(void* precord)
{

}


static long el33xx_init_record(void* precord)
{
	aiRecord* prec = static_cast<aiRecord*>(precord);

	prec->dpvt = util::parseAndCreateDpvt(prec->inp.value.instio.string);

	return prec->dpvt ? 0 : 1;
}

static long el33xx_read_record(void* precord)
{

}


