//======================================================//
// Name: devEL7XXX.cpp
// Purpose: Device support for EL7xxx modules (motor control)
// Authors: Jeremy L.
// Date Created: July 17, 2019
//======================================================//

/* EPICS includes */
#include <epicsExport.h>
#include <epicsMath.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsAssert.h>
#include <dbAccess.h>
#include <devSup.h>
#include <alarm.h>
#include <epicsString.h>
#include <dbScan.h>
#include <boRecord.h>
#include <iocsh.h>
#include <callback.h>

#include <drvModbusAsyn.h>
#include <asynPortDriver.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "devEK9000.h"

static long EL7047_dev_report(int after);
static long EL7047_init(int after);
static long EL7047_init_record(void* record);
static long EL7047_get_ioint_info(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt);
static long EL7047_write_record(void* record);

struct
{
	long num;
	DEVSUPFUN report;
	DEVSUPFUN init;
	DEVSUPFUN init_record;
	DEVSUPFUN ioint_info;
	DEVSUPFUN write_record;
} devEL7XXX = {
	5,
	(DEVSUPFUN)EL7047_dev_report,
	(DEVSUPFUN)EL7047_init,
	(DEVSUPFUN)EL7047_init_record,
	(DEVSUPFUN)EL7047_get_ioint_info,
	(DEVSUPFUN)EL7047_write_record,
};


static long EL7047_dev_report(int after)
{
	return 0;
}

static long EL7047_init(int after)
{
	return 0;
}

static long EL7047_init_record(void* record)
{
	return 0;
}

static long EL7047_get_ioint_info(int cmd, struct dbCommon *precord, IOSCANPVT *ppvt)
{
	return 0;
}

static long EL7047_write_record(void* record)
{
	return 0;
}